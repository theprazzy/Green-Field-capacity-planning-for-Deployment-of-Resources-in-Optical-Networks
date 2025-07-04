from net2plan import NetPlan
from net2plan.libraries import GraphUtils
from net2plan.utils import InputParameter
from pyomo.environ import *

def execute_algorithm(netPlan, algorithmParameters, net2planParameters):
    solver_name = algorithmParameters.get("solverName", "glpk")
    solver_library_name = algorithmParameters.get("solverLibraryName", "C:\\glpk-4.48\\w32\\glpk_4_48.dll")
    max_solver_time = float(algorithmParameters.get("maxSolverTimeInSeconds", -1))
    k = 5
    
    netPlan.setRoutingTypeAllDemands("SOURCE_ROUTING")
    netPlan.removeAllRoutes()
    
    for d in netPlan.getDemands():
        k_shortest_paths = GraphUtils.getKLooplessShortestPaths(
            netPlan.getNodes(), netPlan.getLinks(),
            d.getIngressNode(), d.getEgressNode(), None,
            k, -1, -1, -1, -1, -1, -1
        )
        if not k_shortest_paths:
            raise Exception("There are no admissible routes for a demand")
        for sp in k_shortest_paths:
            netPlan.addRoute(d, 0, 0, sp, None)
    
    model = ConcreteModel()
    num_routes = netPlan.getNumberOfRoutes()
    model.x_p = Var(range(num_routes), domain=NonNegativeReals)
    
    l_p = netPlan.getVectorRouteNumberOfLinks()
    model.obj = Objective(expr=sum(l_p[i] * model.x_p[i] for i in range(num_routes)), sense=minimize)
    
    def demand_constraint_rule(model, d_index):
        routes_d = [r.getIndex() for r in netPlan.getDemands()[d_index].getRoutes()]
        h_d = netPlan.getDemands()[d_index].getOfferedTraffic()
        return sum(model.x_p[i] for i in routes_d) == h_d
    
    model.demand_constraints = Constraint(range(len(netPlan.getDemands())), rule=demand_constraint_rule)
    
    def link_capacity_rule(model, e_index):
        routes_e = [r.getIndex() for r in netPlan.getLinks()[e_index].getTraversingRoutes()]
        u_e = netPlan.getLinks()[e_index].getCapacity()
        return sum(model.x_p[i] for i in routes_e) <= u_e
    
    model.link_constraints = Constraint(range(len(netPlan.getLinks())), rule=link_capacity_rule)
    
    solver = SolverFactory(solver_name, executable=solver_library_name)
    results = solver.solve(model, tee=True, timelimit=max_solver_time if max_solver_time > 0 else None)
    
    if results.solver.termination_condition != TerminationCondition.optimal:
        raise Exception("An optimal solution was not found")
    
    x_p_values = {i: model.x_p[i].value for i in range(num_routes)}
    for r in netPlan.getRoutes():
        r.setCarriedTraffic(x_p_values[r.getIndex()], x_p_values[r.getIndex()])
    
    netPlan.removeAllRoutesUnused(0.001)
    
    return f"Ok! Total bandwidth consumed in the links: {model.obj.expr()}"
