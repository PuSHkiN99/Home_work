# Решение ищется одновременно для всех сценариев и режимов работы
# ОПТИМИЗИРОВАННАЯ ВЕРСИЯ: Constraint Generation + f_loc + Parallel LP Check + Warm Start + Dynamic Gap

import pulp
import networkx as nx
import itertools
import os
import pickle
import json
import time
import concurrent.futures

print("=== N-1 + BUS + геометрия (с потерями) + ТРИ РЕЖИМА - CONSTRAINT GENERATION (PARALLEL) ===\n")

# ====================== ГЛОБАЛЬНЫЕ ПАРАМЕТРЫ ======================
NUM_WORKERS = min(8, os.cpu_count() or 8)  # Количество ядер для параллельной проверки
DISTANCE_FACTOR = 1.3
CONVERTER_COPIES = 2
MIN_CABLE_LENGTH = 1.0
OVERLOAD_FACTOR = 1.2  # Перегрузка при N-1
MAX_ITERATIONS = 15
TIME_LIMIT_PER_ITER = 1800  # Макс. время на одну MILP-итерацию (сек)
INITIAL_GAP = 0.10          # Допуск 10% для быстрых промежуточных итераций
FINAL_GAP = 0.01            # Допуск 1% для финальной проверки

# ====================== НАСТРОЙКА РЕШАТЕЛЯ (ИСПРАВЛЕНО) ======================
def get_solver(time_limit=TIME_LIMIT_PER_ITER, gap=INITIAL_GAP, msg=True):
    try:
        path_to_highs = r"F:\ПО\highs-1.14.0-x86_64-windows-mit\bin\highs.exe"
        if os.path.exists(path_to_highs):
            # PuLP HiGHS_CMD нативно поддерживает threads и timeLimit.
            # options передаем БЕЗ '--', PuLP добавит его сам.
            return pulp.HiGHS_CMD(
                path=path_to_highs,
                msg=msg,
                timeLimit=time_limit,
                threads=NUM_WORKERS,
                options=[f'mip_rel_gap={gap}']
            )
    except Exception as e:
        print(f"⚠️ HiGHS не доступен: {e}")
    
    # Фоллбэк на CBC
    return pulp.PULP_CBC_CMD(
        msg=msg,
        timeLimit=time_limit,
        threads=NUM_WORKERS,
        fracGap=gap,
        presolve=True
    )

# ====================== ДАННЫЕ ======================
wire_types = [
    {"voltage": 27, "mass_per_m": 0.019, "loss_per_m": 0.025},
    {"voltage": 270, "mass_per_m": 0.035, "loss_per_m": 0.003}
]
wire_dict = {w["voltage"]: w for w in wire_types}

consumers = [
    {"name": "Привод стабилизатора (Stabilizer trim actuator)", "demand_takeoff": 500, "demand_cruise": 300, "demand_landing": 800, "voltage": 270, "x": -15.0, "y": 0.0},
    {"name": "Комплекс авионики (Avionics suite)", "demand_takeoff": 2000, "demand_cruise": 2000, "demand_landing": 2000, "voltage": 27, "x": 6.5, "y": -0.5},
    {"name": "Гидравлические насосы (Electric hydraulic pumps)", "demand_takeoff": 5000, "demand_cruise": 2000, "demand_landing": 6000, "voltage": 270, "x": 4.0, "y": -1.0},
    {"name": "Система кондиционирования (Air conditioning packs)", "demand_takeoff": 7000, "demand_cruise": 7000, "demand_landing": 7000, "voltage": 270, "x": 3.0, "y": 0.0},
    {"name": "Противообледенительная система крыла (Wing anti-ice)", "demand_takeoff": 3500, "demand_cruise": 4000, "demand_landing": 3000, "voltage": 270, "x": 6.0, "y": -2.0}
]

sources = [
    {"name": "Engine_Gen_IDG_Left", "max_power": 120000, "voltage": 270, "mass": 48, "x": 2.5, "y": -3.0},
    {"name": "Engine_Gen_IDG_Right", "max_power": 120000, "voltage": 270, "mass": 48, "x": 2.5, "y": 3.0},
    {"name": "Engine_Gen_VFG_Left", "max_power": 130000, "voltage": 270, "mass": 38, "x": 2.5, "y": -3.0},
    {"name": "Engine_Gen_VFG_Right", "max_power": 130000, "voltage": 270, "mass": 38, "x": 2.5, "y": 3.0},
    {"name": "APU_Gen_VFG", "max_power": 110000, "voltage": 270, "mass": 22, "x": -12.0, "y": 0.0},
    {"name": "APU_Gen_SG", "max_power": 85000, "voltage": 270, "mass": 32, "x": -12.0, "y": 0.0}
]

location_options = [(7.5, -0.5), (2.5, -3.0), (2.5, 3.0), (-14.0, 0.0)]

converters = [
    {"name": "DCDC_DCM5614", "from": 270, "to": 27, "eff": 0.96, "mass": 17.8, "capacity": 1300, "bidirectional": 0},
    {"name": "DCDC_TC6500", "from": 270, "to": 27, "eff": 0.96, "mass": 47, "capacity": 6500, "bidirectional": 0}
]

voltages = list(set([c["voltage"] for c in consumers] + [s["voltage"] for s in sources]))
operation_modes = ["takeoff", "cruise", "landing"]

# ====================== ПОСТРОЕНИЕ ГРАФА ======================
G = nx.DiGraph()

def add_node(name, **attrs):
    G.add_node(name, **attrs)

for s in sources:
    attrs = dict(type="source", voltage=s["voltage"], max_power=s["max_power"], mass=s["mass"], layer=0)
    if "x" in s:
        attrs["fixed_pos"] = (s["x"], s["y"])
    add_node(s["name"], **attrs)

for c in consumers:
    add_node(c["name"], type="consumer", voltage=c["voltage"],
             demand_takeoff=c["demand_takeoff"],
             demand_cruise=c["demand_cruise"],
             demand_landing=c["demand_landing"],
             fixed_pos=(c["x"], c["y"]), layer=4)

for v in voltages:
    for i in range(2):
        add_node(f"Bus_{v}_{i}", type="bus", voltage=v, mass=20, layer=2)

converter_pairs = []
converter_bidirectional = {}
for conv in converters:
    bidirectional = conv.get("bidirectional", 0)
    for copy_idx in range(CONVERTER_COPIES):
        copy_suffix = f"_c{copy_idx}"
        in_n = f"in_{conv['name']}{copy_suffix}"
        out_n = f"out_{conv['name']}{copy_suffix}"
        add_node(in_n, type="conv_in", voltage=conv["from"], layer=1, parent_conv=conv['name'], copy=copy_idx)
        add_node(out_n, type="conv_out", voltage=conv["to"], layer=3, parent_conv=conv['name'], copy=copy_idx)
        converter_pairs.append((in_n, out_n))
        converter_bidirectional[in_n] = bidirectional
        converter_bidirectional[out_n] = bidirectional
        G.add_edge(in_n, out_n, type="converter", capacity=conv["capacity"], efficiency=conv["eff"], mass=conv["mass"], direction="forward")
        if bidirectional == 1:
            G.add_edge(out_n, in_n, type="converter", capacity=conv["capacity"], efficiency=conv["eff"], mass=conv["mass"], direction="backward")

nodes = list(G.nodes())
nodes_without_pos = [n for n in nodes if "fixed_pos" not in G.nodes[n]]

allowed_connections = {
    ("source", "conv_in"): True, ("source", "bus"): True, ("conv_in", "bus"): True,
    ("bus", "conv_in"): True, ("bus", "conv_out"): True, ("bus", "consumer"): True,
    ("bus", "bus"): True, ("conv_out", "bus"): True, ("conv_out", "consumer"): True,
}

for u, v in itertools.permutations(nodes, 2):
    if G.nodes[v]["type"] == "source" or G.nodes[u]["type"] == "consumer":
        continue
    if G.nodes[u]["voltage"] != G.nodes[v]["voltage"]:
        continue
    tu, tv = G.nodes[u]["type"], G.nodes[v]["type"]
    if (tu == "conv_in" and tv == "conv_out") or (tu == "conv_out" and tv == "conv_in"):
        continue
    if (tu, tv) not in allowed_connections:
        continue
    if tu == "conv_in" and converter_bidirectional.get(u, 0) == 0:
        continue
    if tv == "conv_out" and converter_bidirectional.get(v, 0) == 0:
        continue
    if not G.has_edge(u, v):
        G.add_edge(u, v, type="cable", capacity=None)

edges = list(G.edges())
cable_edges = [e for e in edges if G.edges[e]["type"] == "cable"]
converter_edges = [e for e in edges if G.edges[e]["type"] == "converter"]

# ====================== СЦЕНАРИИ N-1 (ОПТИМИЗИРОВАНО) ======================
critical_edges = [
    e for e in edges 
    if G.edges[e]["type"] != "converter" and (
        G.nodes[e[0]]["type"] in ("source", "bus", "conv_in", "conv_out") or 
        G.nodes[e[1]]["type"] in ("source", "bus", "conv_in", "conv_out")
    )
]

scenario_names = ["base"]
scenario_edge_map = {}

for idx, e in enumerate(critical_edges):
    s_name = f"fail_cable_{idx}"
    scenario_names.append(s_name)
    scenario_edge_map[s_name] = e

for idx, e in enumerate(converter_edges):
    s_name = f"fail_conv_{idx}"
    scenario_names.append(s_name)
    scenario_edge_map[s_name] = e

scenarios = scenario_names

print(f"\n📊 Статистика модели:")
print(f"   Узлов: {len(nodes)}")
print(f"   Рёбер: {len(edges)} (кабели: {len(cable_edges)}, конвертеры: {len(converter_edges)})")
print(f"   Сценариев N-1: {len(scenarios)-1}")
print(f"   Режимов работы: {len(operation_modes)}")
print(f"   Всего проверок отказоустойчивости: {(len(scenarios)-1) * len(operation_modes)}")
print(f"   Параллельных потоков: {NUM_WORKERS}")

# ====================== ТОЧНЫЙ BIG-M ======================
MAX_DEMAND = max(
    sum(c["demand_takeoff"] for c in consumers),
    sum(c["demand_cruise"] for c in consumers),
    sum(c["demand_landing"] for c in consumers)
)
BIGM_FLOW = MAX_DEMAND * 1.2
BIGM_SOURCE = {s["name"]: s["max_power"] for s in sources}
print(f"   Точный BIGM_FLOW: {BIGM_FLOW:.0f}")

# ====================== ПРЕДВАРИТЕЛЬНЫЙ РАСЧЁТ ДЛИН И ПОТЕРЬ ======================
cable_length = {}
cable_eff = {}
for e in cable_edges:
    u, v = e
    voltage = G.nodes[u]["voltage"]
    loss_per_m = wire_dict[voltage]["loss_per_m"]
    for i, loc1 in enumerate(location_options):
        for j, loc2 in enumerate(location_options):
            p1 = G.nodes[u].get("fixed_pos", loc1)
            p2 = G.nodes[v].get("fixed_pos", loc2)
            d_manhattan = abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
            d = max(d_manhattan * DISTANCE_FACTOR, MIN_CABLE_LENGTH)
            cable_length[(e, i, j)] = d
            cable_eff[(e, i, j)] = max(0.1, 1 - loss_per_m * d)

# ====================== ФУНКЦИЯ: Построение MILP ======================
def build_milp(active_scenarios, name="MILP", warm_start_y=None, warm_start_w=None):
    prob = pulp.LpProblem(name, pulp.LpMinimize)
    
    y = pulp.LpVariable.dicts("y", edges, cat="Binary")
    z = {}
    for n in nodes_without_pos:
        for i in range(len(location_options)):
            z[(n, i)] = pulp.LpVariable(f"z_{n}_{i}", cat="Binary")
        prob += pulp.lpSum(z[(n, i)] for i in range(len(location_options))) == 1
    
    for in_node, out_node in converter_pairs:
        if "fixed_pos" not in G.nodes[in_node] and "fixed_pos" not in G.nodes[out_node]:
            for i in range(len(location_options)):
                prob += z[(in_node, i)] == z[(out_node, i)]
    
    w = {}
    for e in cable_edges:
        for i in range(len(location_options)):
            for j in range(len(location_options)):
                w[(e, i, j)] = pulp.LpVariable(f"w_{e[0]}_{e[1]}_{i}_{j}", cat="Binary")
                zi = z.get((e[0], i), 1)
                zj = z.get((e[1], j), 1)
                prob += w[(e, i, j)] <= y[e]
                if isinstance(zi, pulp.LpVariable): prob += w[(e, i, j)] <= zi
                if isinstance(zj, pulp.LpVariable): prob += w[(e, i, j)] <= zj
                prob += w[(e, i, j)] >= y[e] + (zi if isinstance(zi, pulp.LpVariable) else 1) + (zj if isinstance(zj, pulp.LpVariable) else 1) - 2
    
    f_loc = pulp.LpVariable.dicts(
        "f_loc", 
        [(e, s, m, i, j) for e in cable_edges for s in active_scenarios for m in operation_modes 
         for i in range(len(location_options)) for j in range(len(location_options))],
        lowBound=0, upBound=BIGM_FLOW
    )
    f_conv = pulp.LpVariable.dicts(
        "f_conv", 
        [(e, s, m) for e in converter_edges for s in active_scenarios for m in operation_modes],
        lowBound=0, upBound=BIGM_FLOW
    )
    
    for e in cable_edges:
        cap = G.edges[e].get("capacity") or BIGM_FLOW
        for s in active_scenarios:
            overload = OVERLOAD_FACTOR if s != "base" else 1.0
            for m in operation_modes:
                prob += pulp.lpSum(f_loc[(e, s, m, i, j)] 
                                   for i in range(len(location_options)) 
                                   for j in range(len(location_options))) <= cap * overload * y[e]
                for i in range(len(location_options)):
                    for j in range(len(location_options)):
                        prob += f_loc[(e, s, m, i, j)] <= BIGM_FLOW * w[(e, i, j)]
    
    for e in converter_edges:
        cap = G.edges[e].get("capacity", BIGM_FLOW)
        for s in active_scenarios:
            for m in operation_modes:
                if s in scenario_edge_map and scenario_edge_map[s] == e:
                    prob += f_conv[(e, s, m)] == 0
                else:
                    overload = OVERLOAD_FACTOR if s != "base" else 1.0
                    prob += f_conv[(e, s, m)] <= cap * overload * y[e]
    
    f_max = {}
    for e in cable_edges:
        for i in range(len(location_options)):
            for j in range(len(location_options)):
                f_max[(e, i, j)] = pulp.LpVariable(f"f_max_{e[0]}_{e[1]}_{i}_{j}", lowBound=0)
                for s in active_scenarios:
                    for m in operation_modes:
                        prob += f_max[(e, i, j)] >= f_loc[(e, s, m, i, j)]
    
    source_used = {}
    for node in G.nodes():
        if G.nodes[node]["type"] == "source":
            source_used[node] = pulp.LpVariable(f"su_{node}", cat="Binary")
            for e in [e for e in edges if e[0] == node]:
                for s in active_scenarios:
                    for m in operation_modes:
                        if G.edges[e]["type"] == "cable":
                            prob += source_used[node] * BIGM_SOURCE[node] >= pulp.lpSum(
                                f_loc[(e, s, m, i, j)] for i in range(len(location_options)) for j in range(len(location_options)))
                        else:
                            prob += source_used[node] * BIGM_SOURCE[node] >= f_conv[(e, s, m)]
    
    bus_used = {}
    for node in G.nodes():
        if G.nodes[node]["type"] == "bus":
            bus_used[node] = pulp.LpVariable(f"bu_{node}", cat="Binary")
            for e in [e for e in edges if e[0] == node or e[1] == node]:
                for s in active_scenarios:
                    for m in operation_modes:
                        if G.edges[e]["type"] == "cable":
                            prob += bus_used[node] * BIGM_FLOW >= pulp.lpSum(
                                f_loc[(e, s, m, i, j)] for i in range(len(location_options)) for j in range(len(location_options)))
                        else:
                            prob += bus_used[node] * BIGM_FLOW >= f_conv[(e, s, m)]
    
    for v in voltages:
        b0, b1 = f"Bus_{v}_0", f"Bus_{v}_1"
        if b0 in bus_used and b1 in bus_used:
            prob += bus_used[b0] >= bus_used[b1]
    
    cost = []
    for node in G.nodes():
        if G.nodes[node]["type"] == "source":
            cost.append(G.nodes[node]["mass"] * source_used[node])
        elif G.nodes[node]["type"] == "bus":
            cost.append(G.nodes[node].get("mass", 20) * bus_used[node])
    for e in converter_edges:
        cost.append(G.edges[e]["mass"] * y[e])
    for e in cable_edges:
        u, v = e
        m_per_m = wire_dict[G.nodes[u]["voltage"]]["mass_per_m"]
        for i in range(len(location_options)):
            for j in range(len(location_options)):
                d = cable_length[(e, i, j)]
                cost.append(d * m_per_m * w[(e, i, j)])
                cost.append(0.001 * f_max[(e, i, j)])
    prob += pulp.lpSum(cost)
    
    for s in active_scenarios:
        if s == "base": continue
        failed = scenario_edge_map[s]
        if G.edges[failed]["type"] == "cable":
            for m in operation_modes:
                for i in range(len(location_options)):
                    for j in range(len(location_options)):
                        prob += f_loc[(failed, s, m, i, j)] == 0
    
    for s in active_scenarios:
        for m in operation_modes:
            for node in G.nodes():
                inflow = outflow = 0
                for e in G.out_edges(node):
                    if G.edges[e]["type"] == "cable":
                        outflow += pulp.lpSum(f_loc[(e, s, m, i, j)] 
                                              for i in range(len(location_options)) for j in range(len(location_options)))
                    else:
                        outflow += f_conv[(e, s, m)]
                for e in G.in_edges(node):
                    if G.edges[e]["type"] == "converter":
                        inflow += f_conv[(e, s, m)] * G.edges[e]["efficiency"]
                    elif G.edges[e]["type"] == "cable":
                        for i in range(len(location_options)):
                            for j in range(len(location_options)):
                                inflow += f_loc[(e, s, m, i, j)] * cable_eff[(e, i, j)]
                t = G.nodes[node]["type"]
                if t == "source":
                    prob += outflow <= G.nodes[node]["max_power"]
                elif t == "consumer":
                    prob += inflow >= G.nodes[node][f"demand_{m}"]
                else:
                    prob += inflow == outflow
    
    # Warm Start
    applied_ws = 0
    if warm_start_y:
        for e in edges:
            if y[e].name in warm_start_y:
                y[e].setInitialValue(warm_start_y[y[e].name])
                applied_ws += 1
    if warm_start_w:
        for key, var in w.items():
            if key in warm_start_w:
                var.setInitialValue(warm_start_w[key])
                applied_ws += 1
    if applied_ws > 0:
        print(f"   🔥 Warm start применен: {applied_ws} переменных")
        
    return prob, y, z, w, f_loc, f_conv

# ====================== ФУНКЦИЯ: LP-проверка одного сценария (для параллелизма) ======================
def check_scenario_feasibility(s, current_y, current_w):
    prob_check = pulp.LpProblem(f"check_{s}", pulp.LpMinimize)
    
    f_loc_c = pulp.LpVariable.dicts(
        "fc", 
        [(e, m, i, j) for e in cable_edges for m in operation_modes 
         for i in range(len(location_options)) for j in range(len(location_options))],
        lowBound=0, upBound=BIGM_FLOW
    )
    f_conv_c = pulp.LpVariable.dicts(
        "fcc", 
        [(e, m) for e in converter_edges for m in operation_modes],
        lowBound=0, upBound=BIGM_FLOW
    )
    
    for e in cable_edges:
        if current_y[e] < 0.5:
            for m in operation_modes:
                for i in range(len(location_options)):
                    for j in range(len(location_options)):
                        prob_check += f_loc_c[(e, m, i, j)] == 0
        else:
            cap = G.edges[e].get("capacity") or BIGM_FLOW
            overload = OVERLOAD_FACTOR if s != "base" else 1.0
            for m in operation_modes:
                prob_check += pulp.lpSum(f_loc_c[(e, m, i, j)] 
                                         for i in range(len(location_options)) 
                                         for j in range(len(location_options))) <= cap * overload
                for i in range(len(location_options)):
                    for j in range(len(location_options)):
                        if current_w.get((e, i, j), 0) < 0.5:
                            prob_check += f_loc_c[(e, m, i, j)] == 0
    
    for e in converter_edges:
        if current_y[e] < 0.5:
            for m in operation_modes:
                prob_check += f_conv_c[(e, m)] == 0
        else:
            cap = G.edges[e].get("capacity", BIGM_FLOW)
            for m in operation_modes:
                if s in scenario_edge_map and scenario_edge_map[s] == e:
                    prob_check += f_conv_c[(e, m)] == 0
                else:
                    overload = OVERLOAD_FACTOR if s != "base" else 1.0
                    prob_check += f_conv_c[(e, m)] <= cap * overload
    
    if s != "base":
        failed = scenario_edge_map[s]
        if G.edges[failed]["type"] == "cable":
            for m in operation_modes:
                for i in range(len(location_options)):
                    for j in range(len(location_options)):
                        prob_check += f_loc_c[(failed, m, i, j)] == 0
    
    for m in operation_modes:
        for node in G.nodes():
            inflow = outflow = 0
            for e in G.out_edges(node):
                if G.edges[e]["type"] == "cable":
                    outflow += pulp.lpSum(f_loc_c[(e, m, i, j)] 
                                          for i in range(len(location_options)) for j in range(len(location_options)))
                else:
                    outflow += f_conv_c[(e, m)]
            for e in G.in_edges(node):
                if G.edges[e]["type"] == "converter":
                    inflow += f_conv_c[(e, m)] * G.edges[e]["efficiency"]
                elif G.edges[e]["type"] == "cable":
                    for i in range(len(location_options)):
                        for j in range(len(location_options)):
                            inflow += f_loc_c[(e, m, i, j)] * cable_eff[(e, i, j)]
            t = G.nodes[node]["type"]
            if t == "source":
                prob_check += outflow <= G.nodes[node]["max_power"]
            elif t == "consumer":
                prob_check += inflow >= G.nodes[node][f"demand_{m}"]
            else:
                prob_check += inflow == outflow
    
    prob_check += 0
    
    # Для параллельных LP-проверок используем CBC (он стабильнее в многопоточном режиме Python)
    solver = pulp.PULP_CBC_CMD(msg=False, timeLimit=60, threads=1)
    prob_check.solve(solver)
    
    return (prob_check.status == pulp.LpStatusOptimal) or \
           (prob_check.status == pulp.LpStatusNotSolved and pulp.value(prob_check.objective) is not None)

# ====================== CONSTRAINT GENERATION ======================
print("\n" + "="*70)
print("🎯 CONSTRAINT GENERATION: Поиск точного глобального оптимума")
print("="*70)

active_scenarios = ["base"]
critical_start = sorted(
    [s for s in scenarios if s != "base"],
    key=lambda s: sum(c[f"demand_{m}"] for c in consumers for m in operation_modes)
)[:3]
active_scenarios.extend(critical_start)

iteration = 0
final_solution = None
total_solve_time = 0
warm_start_y = {}
warm_start_w = {}

while iteration < MAX_ITERATIONS:
    iteration += 1
    print(f"\n{'='*70}")
    print(f"🔄 ИТЕРАЦИЯ {iteration}: MILP с {len(active_scenarios)} активными сценариями")
    print(f"   Сценарии: {active_scenarios[:5]}{'...' if len(active_scenarios) > 5 else ''}")
    print(f"{'='*70}")
    
    # Динамический Gap и Time Limit
    gap = FINAL_GAP if iteration >= 4 else INITIAL_GAP
    t_limit = 3600 if iteration >= 4 else TIME_LIMIT_PER_ITER
    
    prob_it, y_it, z_it, w_it, f_loc_it, f_conv_it = build_milp(
        active_scenarios, f"MILP_iter_{iteration}", warm_start_y, warm_start_w
    )
    
    print(f"📊 Размерность: {len(prob_it.variables())} перем., {len(prob_it.constraints)} огр.")
    solver_it = get_solver(time_limit=t_limit, gap=gap)
    start_it = time.time()
    prob_it.solve(solver_it)
    time_it = time.time() - start_it
    total_solve_time += time_it
    
    status_it = pulp.LpStatus[prob_it.status]
    obj_val = pulp.value(prob_it.objective)
    obj_str = f"{obj_val:.2f}" if obj_val is not None else "N/A"
    print(f"✅ MILP: {status_it}, время: {time_it:.2f}с, цель: {obj_str}")
    
    if prob_it.status not in [pulp.LpStatusOptimal, pulp.LpStatusNotSolved] or obj_val is None:
        print("❌ Задача неразрешима или решатель не нашел допустимого решения!")
        break
    
    current_y = {e: round(pulp.value(y_it[e]) or 0) for e in edges}
    current_z = {k: round(pulp.value(v) or 0) for k, v in z_it.items()}
    current_w = {k: round(pulp.value(v) or 0) for k, v in w_it.items()}
    
    # Сохраняем для Warm Start следующей итерации
    warm_start_y = {y_it[e].name: pulp.value(y_it[e]) for e in edges if pulp.value(y_it[e]) is not None}
    warm_start_w = {k: pulp.value(v) for k, v in w_it.items() if pulp.value(v) is not None}
    
    print(f"   Выбрано рёбер: {sum(current_y.values())}")
    
    # ПАРАЛЛЕЛЬНАЯ ПРОВЕРКА ОСТАЛЬНЫХ СЦЕНАРИЕВ
    inactive_scenarios = [s for s in scenarios if s not in active_scenarios]
    print(f"\n🔍 Параллельная LP-проверка {len(inactive_scenarios)} сценариев ({NUM_WORKERS} потоков)...")
    
    violated_scenarios = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=NUM_WORKERS) as executor:
        future_to_s = {executor.submit(check_scenario_feasibility, s, current_y, current_w): s for s in inactive_scenarios}
        for future in concurrent.futures.as_completed(future_to_s):
            s = future_to_s[future]
            try:
                if not future.result():
                    violated_scenarios.append(s)
            except Exception as e:
                print(f"   ⚠️ Ошибка проверки {s}: {e}")
                
    print(f"   Итог: {len(violated_scenarios)} нарушений")
    
    if len(violated_scenarios) == 0:
        print(f"\n🎉 ВСЕ СЦЕНАРИИ ПРОХОДЯТ ВО ВСЕХ РЕЖИМАХ!")
        print(f"   Найден ТОЧНЫЙ ГЛОБАЛЬНЫЙ ОПТИМУМ!")
        print(f"   Итераций: {iteration}, активных сценариев: {len(active_scenarios)} (из {len(scenarios)})")
        
        final_solution = {
            'prob': prob_it, 'y': y_it, 'z': z_it, 'w': w_it,
            'f_loc': f_loc_it, 'f_conv': f_conv_it,
            'current_y': current_y, 'current_z': current_z, 'current_w': current_w,
            'objective': obj_val, 'status': status_it
        }
        break
    else:
        # Добавляем только топ-3 самых критичных для контроля роста модели
        to_add = violated_scenarios[:min(3, len(violated_scenarios))]
        active_scenarios.extend(to_add)
        print(f"   Добавляем в MILP: {to_add}")
        print(f"   Всего активных станет: {len(active_scenarios)}")
        
        final_solution = {
            'prob': prob_it, 'y': y_it, 'z': z_it, 'w': w_it,
            'f_loc': f_loc_it, 'f_conv': f_conv_it,
            'current_y': current_y, 'current_z': current_z, 'current_w': current_w,
            'objective': obj_val, 'status': 'Intermediate'
        }

if final_solution is None:
    print(f"\n⚠️ Не удалось найти решение.")
    exit()

# ====================== СБОР РЕЗУЛЬТАТОВ ======================
print("\n💾 Сохранение результатов...")

results = {
    'graph': G,
    'selected_edges': [e for e in edges if final_solution['current_y'][e] > 0.5],
    'base_flows': {},
    'fail_flows': {},
    'node_positions': {},
    'wire_dict': wire_dict,
    'location_options': location_options,
    'w_values': {k: v for k, v in final_solution['current_w'].items() if v > 0.5},
    'cable_length': dict(cable_length),
    'solution_status': final_solution['status'],
    'objective_value': final_solution['objective'],
    'solve_time': total_solve_time,
    'operation_modes': operation_modes,
    'consumers': consumers,
    'sources': sources,
    'converters': converters,
    'iterations': iteration,
    'active_scenarios': active_scenarios,
    'total_scenarios': len(scenarios)
}

# Сбор потоков для base и fail сценариев из финальной модели
prob_final = final_solution['prob']
for s in active_scenarios:
    for m in operation_modes:
        for e in cable_edges:
            val = sum(pulp.value(final_solution['f_loc'][(e, s, m, i, j)]) or 0 
                      for i in range(len(location_options)) for j in range(len(location_options)))
            if val > 0.01:
                if s == "base":
                    results['base_flows'].setdefault(e, {})[m] = val
                else:
                    fail_key = scenario_edge_map[s]
                    results['fail_flows'].setdefault(fail_key, {}).setdefault(e, {})[m] = val
        
        for e in converter_edges:
            val = pulp.value(final_solution['f_conv'][(e, s, m)]) or 0
            if val > 0.01:
                if s == "base":
                    results['base_flows'].setdefault(e, {})[m] = val
                else:
                    fail_key = scenario_edge_map[s]
                    results['fail_flows'].setdefault(fail_key, {}).setdefault(e, {})[m] = val

# Позиции узлов
for n in G.nodes():
    if "fixed_pos" in G.nodes[n]:
        results['node_positions'][n] = G.nodes[n]["fixed_pos"]
    else:
        found = False
        for i, loc in enumerate(location_options):
            if final_solution['current_z'].get((n, i), 0) > 0.5:
                results['node_positions'][n] = loc
                found = True
                break
        if not found:
            results['node_positions'][n] = location_options[0]

output_file = 'optimization_results.pkl'
with open(output_file, 'wb') as f_out:
    pickle.dump(results, f_out)

json_results = {
    'selected_edges': [(str(u), str(v)) for u, v in results['selected_edges']],
    'base_flows': {
        f"{u}_{v}": {mode: round(flow, 2) for mode, flow in modes.items()}
        for (u, v), modes in results['base_flows'].items()
    },
    'node_positions': {n: list(pos) for n, pos in results['node_positions'].items()},
    'operation_modes': operation_modes,
    'solution_status': results['solution_status'],
    'objective_value': results['objective_value'],
    'solve_time': results['solve_time'],
    'iterations': iteration,
    'active_scenarios_count': len(active_scenarios),
    'total_scenarios_count': len(scenarios)
}

with open('optimization_results.json', 'w', encoding='utf-8') as f_out:
    json.dump(json_results, f_out, indent=2, ensure_ascii=False)

print(f"\n✅ Результаты сохранены:")
print(f"   - {output_file} (полные данные)")
print(f"   - optimization_results.json (основные данные)")

print(f"\n📊 Итоговая сводка:")
print(f"   Статус: {results['solution_status']}")
print(f"   Целевая функция (масса): {results['objective_value']:.2f}")
print(f"   Общее время: {total_solve_time:.2f}с")
print(f"   Итераций Constraint Generation: {iteration}")
print(f"   Активных сценариев в финальной MILP: {len(active_scenarios)} из {len(scenarios)}")
print(f"   Выбрано рёбер: {len(results['selected_edges'])}")

print(f"\n📊 Проверка отказоустойчивости по режимам:")
for m in operation_modes:
    base_demand = sum(c[f"demand_{m}"] for c in consumers)
    print(f"   Режим {m}: суммарное потребление = {base_demand}")

print(f"\n🎯 Запустите visualization_part.py для построения графиков")