#!/usr/bin/env python3
"""
초기화 검증 스크립트
- 서스펜션 평형값 확인
- E-Corner 초기화 확인
- VehicleBody 평형 조건 확인
"""

import numpy as np
from pathlib import Path
import sys

# 프로젝트 루트 추가
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from vehicle_sim.models.vehicle_body.vehicle_body import VehicleBody, VehicleBodyParameters
from vehicle_sim.utils.config_loader import load_param

def main():
    print("=" * 80)
    print("초기화 검증 테스트")
    print("=" * 80)

    # YAML 파라미터 로드
    config_path = project_root / "vehicle_sim" / "models" / "params" / "vehicle_standard.yaml"

    body_param = load_param('vehicle_body', str(config_path))
    geom_param = load_param('vehicle_spec', str(config_path))['geometry']
    susp_param = load_param('suspension', str(config_path))
    physics_param = load_param('physics', str(config_path))

    # VehicleBodyParameters 생성
    params = VehicleBodyParameters(
        m=float(body_param['m']),
        Ixx=float(body_param['inertia']['Ixx']),
        Iyy=float(body_param['inertia']['Iyy']),
        Izz=float(body_param['inertia']['Izz']),
        Ixz=float(body_param['inertia']['Ixz']),
        h_CG=float(susp_param['z_CG0']),
        a=float(geom_param['L_wheelbase']) / 2.0,
        b=float(geom_param['L_wheelbase']) / 2.0,
        L_track=float(geom_param['L_track']),
        L_wheelbase=float(geom_param['L_wheelbase']),
        g=float(physics_param['g'])
    )

    print(f"\n{'[파라미터]':-^80}")
    print(f"m_s (차체 질량): {params.m} kg")
    print(f"m_u (언스프렁 질량): 50.0 kg (각 코너)")
    print(f"K_spring: {susp_param['K_spring']} N/m")
    print(f"z_CG0: {susp_param['z_CG0']} m")
    print(f"delta_s_min: {susp_param['delta_s_min']*1000:.0f} mm")
    print(f"delta_s_max: {susp_param['delta_s_max']*1000:.0f} mm")

    # VehicleBody 생성
    print(f"\n{'[VehicleBody 초기화]':-^80}")
    vehicle = VehicleBody(parameters=params, config_path=str(config_path))
    print("✓ VehicleBody 생성 완료")

    # 1. 서스펜션 내부 상태 확인 (초기화 직후)
    print(f"\n{'[서스펜션 초기 상태 - __init__ 직후]':-^80}")
    print(f"{'Corner':<8} {'F_sus [N]':>12} {'F_tire [N]':>12} {'delta_s [mm]':>12} {'delta_t [mm]':>12}")
    print("-" * 80)

    total_F_sus_init = 0.0
    total_F_tire_init = 0.0

    for label in vehicle.wheel_labels:
        sus_state = vehicle.corners[label].suspension.get_state()
        F_sus = sus_state['F_sus']
        F_tire = sus_state['F_z_tire']
        delta_s = sus_state['delta_s'] * 1000  # mm
        delta_t = sus_state['delta_t'] * 1000  # mm

        total_F_sus_init += F_sus
        total_F_tire_init += F_tire

        print(f"{label:<8} {F_sus:>12.2f} {F_tire:>12.2f} {delta_s:>12.2f} {delta_t:>12.2f}")

    print("-" * 80)
    print(f"{'합계':<8} {total_F_sus_init:>12.2f} {total_F_tire_init:>12.2f}")

    # 2. 평형 조건 계산
    print(f"\n{'[평형 조건 검증]':-^80}")
    m_total = params.m + 4 * 50.0  # sprung + 4 * unsprung
    F_sus_required = params.m * params.g
    F_tire_required = m_total * params.g

    print(f"필요한 Σ F_sus = m_s × g = {params.m} × {params.g} = {F_sus_required:.2f} N")
    print(f"실제 Σ F_sus = {total_F_sus_init:.2f} N")
    print(f"차이 = {total_F_sus_init - F_sus_required:.3f} N ({abs(total_F_sus_init - F_sus_required)/F_sus_required*100:.4f}%)")

    if abs(total_F_sus_init - F_sus_required) < 1.0:
        print("✓ F_sus 평형 조건 만족!")
    else:
        print("✗ F_sus 평형 조건 불만족!")

    print(f"\n필요한 Σ F_tire = m_total × g = {m_total} × {params.g} = {F_tire_required:.2f} N")
    print(f"실제 Σ F_tire = {total_F_tire_init:.2f} N")
    print(f"차이 = {total_F_tire_init - F_tire_required:.3f} N ({abs(total_F_tire_init - F_tire_required)/F_tire_required*100:.4f}%)")

    if abs(total_F_tire_init - F_tire_required) < 1.0:
        print("✓ F_tire 평형 조건 만족!")
    else:
        print("✗ F_tire 평형 조건 불만족!")

    # 3. 언스프렁 동역학 평형 확인
    print(f"\n{'[언스프렁 동역학 평형 - 각 코너]':-^80}")
    print(f"{'Corner':<8} {'F_tire [N]':>12} {'F_sus [N]':>12} {'m_u×g [N]':>12} {'F_net [N]':>12}")
    print("-" * 80)

    all_balanced = True
    for label in vehicle.wheel_labels:
        sus_state = vehicle.corners[label].suspension.get_state()
        m_u = vehicle.corners[label].suspension.unsprung_params.m_u
        g = vehicle.corners[label].suspension.unsprung_params.g

        F_tire = sus_state['F_z_tire']
        F_sus = sus_state['F_sus']
        F_gravity = m_u * g
        F_net = F_tire - F_sus - F_gravity

        print(f"{label:<8} {F_tire:>12.2f} {F_sus:>12.2f} {F_gravity:>12.2f} {F_net:>12.3f}")

        if abs(F_net) > 1.0:
            all_balanced = False

    if all_balanced:
        print("✓ 모든 코너 언스프렁 평형 만족 (F_net ≈ 0)!")
    else:
        print("✗ 언스프렁 평형 불만족!")

    # 4. VehicleBody.reset() 테스트
    print(f"\n{'[VehicleBody.reset() 후 상태]':-^80}")
    vehicle.reset()
    print("✓ vehicle.reset() 호출 완료")

    print(f"\n차체 상태:")
    print(f"  heave = {vehicle.state.heave} m")
    print(f"  roll = {vehicle.state.roll} rad")
    print(f"  pitch = {vehicle.state.pitch} rad")

    print(f"\n서스펜션 상태 (reset 후):")
    print(f"{'Corner':<8} {'F_sus [N]':>12} {'delta_s [mm]':>12}")
    print("-" * 80)

    total_F_sus_reset = 0.0
    for label in vehicle.wheel_labels:
        sus_state = vehicle.corners[label].suspension.get_state()
        F_sus = sus_state['F_sus']
        delta_s = sus_state['delta_s'] * 1000
        total_F_sus_reset += F_sus
        print(f"{label:<8} {F_sus:>12.2f} {delta_s:>12.2f}")

    print("-" * 80)
    print(f"{'합계':<8} {total_F_sus_reset:>12.2f}")
    print(f"필요값: {F_sus_required:.2f} N")
    print(f"차이: {total_F_sus_reset - F_sus_required:.3f} N")

    if abs(total_F_sus_reset - F_sus_required) < 1.0:
        print("✓ reset 후에도 평형 유지!")
    else:
        print("✗ reset 후 평형 깨짐!")

    # 5. 첫 update() 호출 테스트
    print(f"\n{'[첫 update() 호출 테스트]':-^80}")

    dt = 0.001  # 1ms
    corner_inputs = {
        label: {
            "T_steer": 0.0,
            "T_brk": 0.0,
            "T_Drv": 0.0,
            "T_susp": 0.0,
            "z_road": 0.0
        }
        for label in vehicle.wheel_labels
    }

    # update 전 차체 상태 저장
    heave_before = vehicle.state.heave
    heave_dot_before = vehicle.state.heave_dot

    vehicle.update(dt, corner_inputs)

    # update 후 상태
    heave_after = vehicle.state.heave
    heave_dot_after = vehicle.state.heave_dot

    print(f"\nupdate 전후 비교:")
    print(f"  heave: {heave_before:.6f} m → {heave_after:.6f} m (변화: {(heave_after-heave_before)*1000:.3f} mm)")
    print(f"  heave_dot: {heave_dot_before:.6f} m/s → {heave_dot_after:.6f} m/s (변화: {(heave_dot_after-heave_dot_before)*1000:.3f} mm/s)")

    # F_sus 확인
    print(f"\nupdate 후 F_sus:")
    print(f"{'Corner':<8} {'F_sus [N]':>12}")
    print("-" * 80)

    total_F_sus_after = 0.0
    for label in vehicle.wheel_labels:
        corner_state = vehicle.corners[label].get_state()
        F_sus = corner_state['F_sus']
        total_F_sus_after += F_sus
        print(f"{label:<8} {F_sus:>12.2f}")

    print("-" * 80)
    print(f"{'합계':<8} {total_F_sus_after:>12.2f}")
    print(f"필요값: {F_sus_required:.2f} N")
    print(f"차이: {total_F_sus_after - F_sus_required:.3f} N")

    # 6. 10번 update 후 drift 확인
    print(f"\n{'[Drift 테스트 - 100 steps]':-^80}")

    heave_history = [vehicle.state.heave]

    for i in range(100):
        vehicle.update(dt, corner_inputs)
        heave_history.append(vehicle.state.heave)

    heave_final = vehicle.state.heave
    heave_drift = heave_final - heave_history[0]

    print(f"초기 heave: {heave_history[0]*1000:.6f} mm")
    print(f"최종 heave (100 steps 후): {heave_final*1000:.6f} mm")
    print(f"Drift: {heave_drift*1000:.6f} mm")

    if abs(heave_drift) < 0.001:  # 1mm 이내
        print("✓ Drift 없음 - 평형 안정!")
    else:
        print(f"✗ Drift 발생: {heave_drift*1000:.3f} mm")

    # 최종 결과
    print(f"\n{'[최종 결과]':-^80}")

    checks = []
    checks.append(("초기 F_sus 평형", abs(total_F_sus_init - F_sus_required) < 1.0))
    checks.append(("초기 F_tire 평형", abs(total_F_tire_init - F_tire_required) < 1.0))
    checks.append(("언스프렁 평형", all_balanced))
    checks.append(("reset 후 평형 유지", abs(total_F_sus_reset - F_sus_required) < 1.0))
    checks.append(("update 후 평형 유지", abs(total_F_sus_after - F_sus_required) < 1.0))
    checks.append(("Drift 안정성", abs(heave_drift) < 0.001))

    all_pass = all(check[1] for check in checks)

    for name, passed in checks:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status:<8} {name}")

    print("-" * 80)
    if all_pass:
        print("✓✓✓ 모든 테스트 통과! 초기화 완벽!")
    else:
        print("✗✗✗ 일부 테스트 실패 - 수정 필요")

    return 0 if all_pass else 1

if __name__ == "__main__":
    exit(main())
