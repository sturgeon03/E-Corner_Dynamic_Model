# Yaw Rate Steering Controller

??臾몄꽌??`vehicle_sim/controllers/README.md`???곸쐞 媛쒖슂瑜?諛섎났?섏? ?딄퀬,  
`yaw_rate_steering_controller`瑜??ㅼ젣濡?遺숈뿬???곕뒗 諛⑸쾿??吏묒쨷???ъ슜 媛?대뱶?낅땲??

## ??臾몄꽌??踰붿쐞

- ?곸쐞 臾몄꽌?먯꽌 ?대? ?ㅻ챸???대뜑 ?몃━/紐⑤뱢 媛쒖슂???앸왂
- ?ш린?쒕뒗 ?꾨옒 3媛吏留??ㅻ８
  - ?낅젰/異쒕젰 怨꾩빟???ㅼ궗??愿?먯뿉??紐낇솗???뺣━
  - 媛앹껜 API/?⑥닔 API瑜??몄젣 ?대뼸寃??곕뒗吏 ?뺣━
  - YAML 紐⑤뱶 ?ㅼ젙, 鍮좊Ⅸ 寃利??명듃遺? ?몃윭釉붿뒋???뺣━

## 鍮좊Ⅸ ?쒖옉

### 1) import 洹쒖튃

??긽 ?⑦궎吏 猷⑦듃?먯꽌 import ?섏꽭??

```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    YawRateSteeringController,
    YawRateSteeringControllerOptions,
    compute_steering_torque,
    compute_steering_angle,
)
```

`control_blocks/`, `estimators/` ?대? ?뚯씪 吏곸젒 import??鍮꾧텒?μ엯?덈떎.

### 2) 媛앹껜 API濡??쒖옉

```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    YawRateSteeringController,
    YawRateSteeringControllerOptions,
)

options = YawRateSteeringControllerOptions(
    dt=0.01,
    enable_yaw_feedback=True,
    enable_fy_feedback=False,
    enable_steer_feedback=True,
    enable_estimator=False,
)

controller = YawRateSteeringController(options)
```

### 3) ???ㅽ뀦 ?몄텧

```python
state = {
    "yaw_rate": 0.05,
    "vx": 8.0,
    "steering_angle": {"FL": 0.0, "FR": 0.0, "RR": 0.0, "RL": 0.0},
    "fy_tire": {"FL": 0.0, "FR": 0.0, "RR": 0.0, "RL": 0.0},
    "fx_tire": {"FL": 0.0, "FR": 0.0, "RR": 0.0, "RL": 0.0},
    "ay": 0.0,
}
ref = {"yaw_rate": 0.2}

torque_cmd = controller.compute_torque_command(state, ref)
angle_cmd = controller.compute_angle_command(state, ref)
```

## ?낅젰/異쒕젰 怨꾩빟 (?ㅼ궗??湲곗?)

### ?꾩닔 `state`

| ??| ???| ?⑥쐞 | ?ㅻ챸 |
|---|---|---|---|
| `yaw_rate` | `float` | rad/s | ?꾩옱 yaw rate 痢≪젙媛?|
| `vx` | `float` | m/s | ?꾩옱 醫낅갑???띾룄 |
| `steering_angle` | `dict` | rad | 媛?諛뷀?議고뼢媛?(`FL/FR/RR/RL`) |

### ?꾩닔 `ref`

| ??| ???| ?⑥쐞 | ?ㅻ챸 |
|---|---|---|---|
| `yaw_rate` | `float` | rad/s | 紐⑺몴 yaw rate |

### ?좏깮 `state` (?듭뀡???곕씪 ?ъ슜)

| ??| ?ъ슜 議곌굔 | ?ㅻ챸 |
|---|---|---|
| `fy_tire` | Fy feedback ?먮뒗 異붿젙湲?寃利?| 諛뷀대퀎 ?〓젰 痢≪젙媛?|
| `fx_tire` | yaw moment allocator ?낅젰 | 諛뷀대퀎 醫낅젰 痢≪젙媛?|
| `ay` | `fy_feedback_source=estimate` ?먮뒗 slip 異붿젙 | ?↔??띾룄 |
| `delta_dot` | 異붿젙湲??ъ슜 ??| 議고뼢媛??띾룄 痢≪젙媛?|
| `steering_torque_axis` | B 異붿젙湲??ъ슜 ??| 異?湲곗? 議고뼢 ?좏겕 |
| `alpha` | slip estimator 誘몄궗????| 諛뷀?slip angle ?낅젰 |

### 異쒕젰

- `compute_torque_command(...)`  
  - `{"FL": ..., "FR": ..., "RR": ..., "RL": ...}` ?뺥깭 議고뼢 紐⑦꽣 ?좏겕 `[N*m]`
- `compute_angle_command(...)`  
  - 媛숈? ??援ъ“??議고뼢媛?紐낅졊 `[rad]`

## API ?좏깮 湲곗?

### 媛앹껜 API (沅뚯옣)

- ?쒖뼱 猷⑦봽?먯꽌 ?곹깭瑜??곗냽?곸쑝濡??좎??댁빞 ?????ъ슜
- ?대? PID/異붿젙湲??곹깭瑜??꾨젅??媛??좎?

```python
torque_cmd = controller.compute_torque_command(state, ref)
error_debug = controller.compute_error_debug(state, ref)
```

### ?⑥닔 API (鍮좊Ⅸ 泥댄겕??

- ?명듃遺??⑥쐞 ?뺤씤泥섎읆 利됱떆 怨꾩궛留??꾩슂?????ъ슜
- `reset=True`濡?怨듭쑀 ?몄뒪?댁뒪 ?곹깭 珥덇린??媛??
```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    compute_steering_torque,
    compute_steering_angle,
)

torque_cmd = compute_steering_torque(state, ref, reset=True)
angle_cmd = compute_steering_angle(state, ref, reset=True)
```

## ?붾쾭洹?異쒕젰 ?댁꽍

`compute_error_debug(...)`??`debug`?먮뒗 ?꾨옒 ?듭떖 ?좏샇媛 ?ㅼ뼱?듬땲??

| ??| ?섎? |
|---|---|
| `Mz_ff`, `Mz_fb`, `Mz_cmd` | yaw moment feedforward/feedback/?⑹꽦 |
| `Fy_total_cmd`, `Fy_cmd` | ?꾩껜/諛뷀대퀎 紐⑺몴 ?〓젰 |
| `delta_cmd` | 諛뷀대퀎 紐⑺몴 議고뼢媛?|
| `T_align_cmd` | ?쇰씪???좏겕 異붿젙 |
| `T_steer_ff_motor`, `T_steer_ff_axis` | feedforward ?좏겕 |
| `estimator` | B, C_alpha 異붿젙 ?곹깭 諛??낅뜲?댄듃 ?щ? |

?쒕떇???뚮뒗 蹂댄넻 `Mz_cmd`, `delta_cmd`, `T_steer_ff_motor`, `estimator`瑜?癒쇱? 遊낅땲??

## YAML ?ㅼ젙?쇰줈 ?ㅽ뻾?섍린

`param/controller_options.example.yaml` 湲곕컲?쇰줈 紐⑤뱶/?듭뀡??遺덈윭?????덉뒿?덈떎.

```python
from vehicle_sim.controllers.yaw_rate_steering_controller import (
    load_controller_runtime_config,
    YawRateSteeringController,
)

runtime_cfg = load_controller_runtime_config(
    "vehicle_sim/controllers/yaw_rate_steering_controller/param/controller_options.example.yaml"
)

controller = YawRateSteeringController(
    options=runtime_cfg.options,
    vehicle_config_path=runtime_cfg.vehicle_config_path,
    gains_path=runtime_cfg.gains_path,
)
```

### 紐⑤뱶 ?붿빟

| 紐⑤뱶 | ?ㅻ챸 | 沅뚯옣 ?곹솴 |
|---|---|---|
| `ff` | feedforward only | 踰좎씠?ㅻ씪???뺤씤 |
| `ff_fb` | yaw/Fy/steer feedback | ?쇰컲 ?쒖뼱 猷⑦봽 |
| `ff_fb_ls` | feedback + 異붿젙湲?| ?뚮씪誘명꽣 蹂??????ㅽ뿕 |

## 寃利?寃쎈줈

### 鍮좊Ⅸ ?ㅽ뻾

```bash
python -m vehicle_sim.controllers.yaw_rate_steering_controller.examples.controller_usage_demo
python vehicle_sim/controllers/yaw_rate_steering_controller/examples/controller_usage_demo.py
```

### ?명듃遺?湲곕컲 ?뚮’ ?뺤씤

- [test_debug/controller_quick_plot.ipynb](./test_debug/controller_quick_plot.ipynb)
- `controller_usage_demo` ?먮쫫???뺤옣???ъ슜 ?덉젣 ?명듃遺?- ?⑥씪 ?몄텧 + ?ъ슜???뺤쓽 `yaw_rate_ref` ?곗냽 ?몄텧 + `Mz_cmd`, `delta_cmd`, `T_steer` ?뚮’ ?뺤씤

## ?먯＜ 諛쒖깮?섎뒗 臾몄젣

### `ModuleNotFoundError: No module named 'vehicle_sim'`

- ??μ냼 猷⑦듃?먯꽌 ?ㅽ뻾?섍굅?? ?꾨옒泥섎읆 紐⑤뱢 ?ㅽ뻾 諛⑹떇???ъ슜?섏꽭??

```bash
python -m vehicle_sim.controllers.yaw_rate_steering_controller.examples.controller_usage_demo
```

### 異쒕젰???嫄곕굹 吏꾨룞????寃쎌슦

1. `controller_gains.yaml`?먯꽌 `steering_pid.kp/kd`瑜?癒쇱? ??떠 ?뺤씤
2. `yaw_rate_pid.ki`瑜???떠 ?곷텇 ?꾩쟻 ?곹뼢 ?뺤씤
3. `dt`? ?ㅼ젣 ?쒖뼱 猷⑦봽 二쇨린媛 留욌뒗吏 ?먭?

### 異붿젙湲??낅뜲?댄듃媛 湲곕?? ?ㅻ? ??
- `options.enable_estimator`, `enable_b_estimator`, `enable_c_alpha_estimator` ?뺤씤
- ?낅젰 `state`??`delta_dot`, `steering_torque_axis`, `alpha` ?쒓났 ?щ? ?뺤씤

