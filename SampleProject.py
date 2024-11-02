from pyomo.environ import*
model=AbstractModel()

#Set
model.B=Set(initialize=[1,2,3,4])
model.N=Set(initialize=[1,2,3,4])
model.L=Set(initialize=[(1,2),(1,3),(3,4)])
model.P=Set(initialize=[(1,2),(1,3),(2,4)])
model.T=RangeSet(1,5)
model.G=Set()

#Parameter
model.Pmax=Param(model.G)
model.Pmin=Param(model.G)
model.cost_a=Param(model.G)
model.cost_b=Param(model.G)
model.cost_c=Param(model.G)
model.SUC=Param(model.G)
model.SDC=Param(model.G)
model.Rup=Param(model.G)
model.Rdn=Param(model.G)
model.GenLoc=Param(model.G)

model.x=Param(model.L)
model.PLmax=Param(model.L)

model.Load1=Param(model.B,model.T, default=0)
model.Pgmax=Param()
model.GHV=Param()
model.eta=Param()
model.Hprice=Param()

model.C=Param(model.P)
model.HLmax=Param(model.P)

model.Load2=Param(model.N,model.T, default=0)
model.Hgmax=Param()
model.Prmin=Param(model.N)
model.Prmax=Param(model.N)
model.Price=Param(model.T)


#Variable

model.Pg=Var(model.T, within=NonNegativeReals)
model.Pt=Var(model.G, model.T, within=NonNegativeReals)
model.Pr=Var(model.N, model.T, within=NonNegativeReals)
model.Hg=Var(model.T, within=NonNegativeReals)
model.Pl=Var(model.L, model.T, within=Reals)
model.SU=Var(model.G, model.T, within=NonNegativeReals)
model.SD=Var(model.G, model.T, within=NonNegativeReals)
model.I=Var(model.G, model.T, within=Binary)
model.Ht=Var(model.G, model.T, within=NonNegativeIntegers)
model.theta=Var(model.B, model.T, within=Reals)
model.Hl=Var(model.P, model.T, within=Reals)



#Objective function
def objective_rule(model):
    return sum(model.Pg[t]*model.Price[t]+model.Hg[t]*model.Hprice+model.SD[g,t]+model.SU[g,t]+model.cost_a[g]+
               model.cost_b[g]*model.Pt[g,t]+model.cost_c[g]*model.Pt[g,t]
               for g in model.G for t in model.T)
model.Costfunction=Objective(rule=objective_rule, sense=minimize)


#Hydrogen turbine generator constraints
def GenMax_rule(model, g, t):
    return (model.Pmax[g]*model.I[g,t] >= model.Pt[g,t])
model.MaxGen_Limit=Constraint(model.G, model.T, rule=GenMax_rule)

def GenMin_rule(model, g, t):
    return (model.Pt[g,t] >= model.Pmin[g]*model.I[g,t])
model.MinGen_Limit=Constraint(model.G, model.T, rule=GenMin_rule)

def startup_rule(model, g, t):
    if t == 1:
        return Constraint.Skip
    else:
        return model.SU[g,t] >= model.SDC[g]*(model.I[g,t]-model.I[g,t-1]) 

model.StartupConstraint = Constraint(model.G, model.T, rule=startup_rule)

def shutdn_rule(model, g, t):
    if t == 1:
        return Constraint.Skip
    else:
        return model.SU[g,t] >= model.SUC[g]*(model.I[g,t-1]-model.I[g,t]) 

model.ShutdnConstraint = Constraint(model.G, model.T, rule=shutdn_rule)

def Rampdn_rule(model, g, t):
    if t == 1:
        return Constraint.Skip
    else:
        return model.Pt[g,t-1]-model.Pt[g,t] <= model.Rdn[g]
model.Rampdn_Limit=Constraint(model.G, model.T, rule=Rampdn_rule)

def Rampup_rule(model, g, t):
    if t == 1:
        return Constraint.Skip
    else:
        return model.Pt[g,t] - model.Pt[g,t-1] <= model.Rup[g]  
model.RampUp_Limit=Constraint(model.G, model.T, rule=Rampup_rule)


def FuelHydrogen_rule(model, g, t):
    return model.Pt[g,t] == model.Ht[g,t]*model.GHV*model.eta
model.FuelHydrogen_Function=Constraint(model.G, model.T, rule=FuelHydrogen_rule)



# DC Power Flow Constraint: Flow calculation based on voltage angle difference and line impedance
def DC_PF_rule(model, i, j, t):
    return model.Pl[i, j, t] == (model.theta[i, t] - model.theta[j, t]) / model.x[i, j]
model.DCPFConstraint = Constraint(model.L, model.T, rule=DC_PF_rule)

# Power Flow Limits Constraint
def power_flow_limit_rule_pos(model, i, j, t):
    return model.Pl[i, j, t] <= model.PLmax[i, j]  
model.PowerFlow_Limit_Pos = Constraint(model.L, model.T, rule=power_flow_limit_rule_pos)

def power_flow_limit_rule_neg(model, i, j, t):
    return -model.Pl[i, j, t] <= model.PLmax[i, j]  
model.PowerFlow_Limit_Neg = Constraint(model.L, model.T, rule=power_flow_limit_rule_neg)


#Load Balance
def Load_balance_rule(model, bus, t):
    # Power flowing into and out of the bus
    flow_in = sum(model.Pl[i, j, t] for (i, j) in model.L if j == bus)
    flow_out = sum(model.Pl[i, j, t] for (i, j) in model.L if i == bus)
    
    # Generation in the bus (if a generator is located there)
    generation_in_bus = sum(model.Pt[g, t] for g in model.G if model.GenLoc[g] == bus)
    
    if bus == 1:
        # Changed model.Pg[bus, t] to model.Pg[t]
        return flow_in - flow_out + generation_in_bus + model.Pg[t] == model.Load1[bus, t] 
    else:
        return flow_in - flow_out + generation_in_bus == model.Load1[bus, t]
    
model.LoadBalanceConstraint = Constraint(model.B, model.T, rule=Load_balance_rule)

# Linearized Steady-State Gas Flow Constraint
def linear_gas_flow_rule(model, n, m, t):
    return model.Hl[n, m, t] == model.C[n, m] * (model.Pr[n, t] - model.Pr[m, t])
model.LinearGasFlowConstraint = Constraint(model.P, model.T, rule=linear_gas_flow_rule)



# Gas Load Balance Constraint at each node (including generators as loads)
def gas_load_balance_rule(model, node, t):
    # Flow into and out of the node
    flow_in = sum(model.Hl[n, m, t] for (n, m) in model.P if n == node)
    flow_out = sum(model.Hl[n, m, t] for (n, m) in model.P if m == node)
    
    # Gas demand and supply at the node
    demand = model.Load2[node, t] + sum(model.Ht[g, t] for g in model.G if g == node)
    
    # Supplier provides gas only at node 1
    supply = model.Hg[t] if node == 1 else 0
    
    # Balance equation
    return supply + flow_in - flow_out == demand
model.GasLoadBalanceConstraint = Constraint(model.N, model.T, rule=gas_load_balance_rule)


# Supplier Gas Flow Limit at Node 1
def supplier_gas_flow_limit_rule(model, t):
    return model.Hg[t] <= model.Hgmax
model.SupplierGasFlowLimit = Constraint(model.T, rule=supplier_gas_flow_limit_rule)


# Pressure Limits 
def pressure_limits_rule_lower(model, node, t):
    # Return the lower bound constraint
    return model.Prmin[node] <= model.Pr[node, t] 

def pressure_limits_rule_upper(model, node, t):
    # Return the upper bound constraint
    return model.Pr[node, t] <= model.Prmax[node]

# Create two separate constraints for the upper and lower bounds
model.PressureLimitsLower = Constraint(model.N, model.T, rule=pressure_limits_rule_lower)
model.PressureLimitsUpper = Constraint(model.N, model.T, rule=pressure_limits_rule_upper)

# Pipeline Capacity Limits on each line
def pipeline_capacity_limits_rule(model, n, m, t):
    return model.Hl[n, m, t] <= model.HLmax[n, m]
model.PipelineCapacityLimits = Constraint(model.P, model.T, rule=pipeline_capacity_limits_rule)

#Create instance
data= DataPortal()
data.load(filename="Newdata.dat", model=model)
instance=model.create_instance(data)
instance.pprint()

#Solveing model
optimizer=SolverFactory("glpk")
optimizer.solve(instance)
instance.display()