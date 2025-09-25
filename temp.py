from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# def create_data():
#     """Stores the data for the problem."""
#     data = {}
#     data["time_matrix"] = [
#         [0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7],
#         [6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14],
#         [9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9],
#         [8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16],
#         [7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14],
#         [3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8],
#         [6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5],
#         [2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10],
#         [3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6],
#         [2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5],
#         [6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4],
#         [6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10],
#         [4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8],
#         [4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6],
#         [5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2],
#         [9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9],
#         [7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0],
#     ]
#     data["time_windows"] = [
#         (0, 5),   # depot
#         (7, 12),  # 1
#         (10, 15), # 2
#         (16, 18), # 3
#         (10, 13), # 4
#         (0, 5),   # 5
#         (5, 10),  # 6
#         (0, 4),   # 7
#         (5, 10),  # 8
#         (0, 3),   # 9
#         (10, 16), # 10
#         (10, 15), # 11
#         (0, 5),   # 12
#         (5, 10),  # 13
#         (7, 8),   # 14
#         (10, 15), # 15
#         (11, 15), # 16
#     ]
#     data["num_vehicles"] = 4
#     data["depot"] = 0
#     return data

def create_data_model():
    data = {}
    # 0: depot, 1..6: customers
    data["time_matrix"] = [
        #  0  1  2  3  4  5  6
        [  0, 7, 9, 8, 6, 5,10],  # 0 depot
        [  7, 0, 4, 3, 6, 8, 7],  # 1
        [  9, 4, 0, 5, 7, 6, 4],  # 2
        [  8, 3, 5, 0, 4, 7, 6],  # 3
        [  6, 6, 7, 4, 0, 3, 5],  # 4
        [  5, 8, 6, 7, 3, 0, 4],  # 5
        [ 10, 7, 4, 6, 5, 4, 0],  # 6
    ]
    # 到達時間窗 (分鐘)
    data["time_windows"] = [
        (0, 10),  # 0 depot：出發窗
        (5, 12),  # 1
        (6, 14),  # 2
        (8, 16),  # 3
        (4, 10),  # 4
        (7, 15),  # 5
        (3,  9),  # 6
    ]
    # 服務時間（分鐘）
    data["service_time"] = [0, 2, 2, 2, 1, 1, 1]
    data["num_vehicles"] = 2
    data["depot"] = 0
    return data

def main():
    """Solve the VRP with time windows."""
    data = create_data_model()

    # Manager / Model
    manager = pywrapcp.RoutingIndexManager(
        len(data["time_matrix"]), data["num_vehicles"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)
    
    # Transit callback（旅行時間）
    def time_callback(from_index, to_index):
        i = manager.IndexToNode(from_index)
        j = manager.IndexToNode(to_index)
        return data["time_matrix"][i][j]  # must be int
    transit_cb = routing.RegisterTransitCallback(time_callback)

    # 目標：路線成本用時間
    routing.SetArcCostEvaluatorOfAllVehicles(transit_cb)
    
    # 時間維度（允許等待、放寬容量）
    time_name = "Time"
    LARGE = 10**6
    routing.AddDimension(
        transit_cb,
        LARGE,  # waiting slack
        LARGE,  # capacity (per-vehicle horizon)
        False,  # don't force start at t=0
        time_name,
    )
    time_dim = routing.GetDimensionOrDie(time_name)

    # 非 depot 節點：套時間窗
    for node, (open_t, close_t) in enumerate(data["time_windows"]):
        if node == data["depot"]:
            continue
        index = manager.NodeToIndex(node)
        time_dim.CumulVar(index).SetRange(open_t, close_t)

    # 起點：套 depot 出發窗
    depot_open, depot_close = data["time_windows"][data["depot"]]
    for v in range(data["num_vehicles"]):
        start_idx = routing.Start(v)
        time_dim.CumulVar(start_idx).SetRange(depot_open, depot_close)

    # 終點：放寬（重要）
    for v in range(data["num_vehicles"]):
        end_idx = routing.End(v)
        time_dim.CumulVar(end_idx).SetRange(0, LARGE)

    # Finalizers：讓 start/end 偏好較小
    for v in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(time_dim.CumulVar(routing.Start(v)))
        routing.AddVariableMinimizedByFinalizer(time_dim.CumulVar(routing.End(v)))

    # 搜尋參數
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.ALL_UNPERFORMED
    )
    search_params.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC
    )
    search_params.time_limit.FromSeconds(10)

    # 求解
    solution = routing.SolveWithParameters(search_params)
    print('solver status: ', routing.status())
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found.")
        
def get_cumul_data(solution, routing, dimension):
    """Return per-route cumulative min/max arrays for a given dimension."""
    cumul_data = []
    for v in range(routing.vehicles()):
        route_data = []
        index = routing.Start(v)
        dim_var = dimension.CumulVar(index)
        route_data.append([solution.Min(dim_var), solution.Max(dim_var)])
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            dim_var = dimension.CumulVar(index)
            route_data.append([solution.Min(dim_var), solution.Max(dim_var)])
        cumul_data.append(route_data)
    return cumul_data

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    time_dimension = routing.GetDimensionOrDie("Time")

    # 也可取得 Min/Max 範圍版本
    cumul_bounds = get_cumul_data(solution, routing, time_dimension)

    total_time_sum = 0
    makespan = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        plan = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            arrive = solution.Value(time_dimension.CumulVar(index))
            plan.append(f"{node} (arrive {arrive}, window={data['time_windows'][node]})")
            index = solution.Value(routing.NextVar(index))
        # End（回到 depot），一般不限制視窗
        node = manager.IndexToNode(index)
        arrive = solution.Value(time_dimension.CumulVar(index))
        plan.append(f"{node} (arrive {arrive})")

        print(f"Route for vehicle {vehicle_id}:\n  " + " -> ".join(plan))
        print(f"  Route time: {arrive}min\n")

        total_time_sum += arrive
        makespan = max(makespan, arrive)

    print(f"Total time (sum of route end times): {total_time_sum}min")
    print(f"Makespan   (max route end time):    {makespan}min")

    # 若想看 Min/Max 範圍，可解除註解：
    # for i, route_bounds in enumerate(cumul_bounds):
    #     print(f"[Bounds] Vehicle {i}: " + " -> ".join(f"({a},{b})" for a,b in route_bounds))
if __name__ == "__main__":
    main()

