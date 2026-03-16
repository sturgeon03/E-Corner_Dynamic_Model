from vehicle_sim.models.vehicle_body.vehicle_body import VehicleBody


CONFIG_PATH_DEFAULT = None  # 노트북에서 CONFIG_PATH를 주입


def setup_vehicle(scenario, config_path: str) -> VehicleBody:
    """차량 모델 초기화 + 시나리오 초기 조건 적용"""
    body = VehicleBody(config_path=config_path)
    scenario.apply_initial_conditions(body)
    return body


def snapshot(body, aarb) -> dict:
    """현재 스텝의 차량 상태 및 제어기 출력 기록"""
    sus = body.corners
    arb = aarb.get_state()
    return {
        "roll"              : body.state.roll,
        "roll_rate"         : body.state.roll_rate,
        "stroke_diff_front" : sus["FL"].suspension.state.delta_s - sus["FR"].suspension.state.delta_s,
        "stroke_diff_rear"  : sus["RL"].suspension.state.delta_s - sus["RR"].suspension.state.delta_s,
        "M_arb_front"       : arb["M_arb_front"],
        "M_arb_rear"        : arb["M_arb_rear"],
    }
