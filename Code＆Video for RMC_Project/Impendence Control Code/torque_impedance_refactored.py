import sys
import os
import time
import threading
import logging

import numpy as np
import pinocchio as pin

from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.ActuatorCyclicClientRpc import ActuatorCyclicClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import Session_pb2, ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.RouterClient import RouterClientSendOptions

# ─────────────────────────── 日志配置 ───────────────────────────
logging.basicConfig(level=logging.INFO,
                    format="[%(asctime)s] %(levelname)s: %(message)s")

# ─────────────────────────── 常量定义 ───────────────────────────
URDF_PATH = "/home/sml123/kinova_ws/src/kinova_gen3lite_control/gen3_lite.urdf"

CONTROL_MODES = {
    "POSITION": ActuatorConfig_pb2.ControlMode.Value("POSITION"),
    "CURRENT" : ActuatorConfig_pb2.ControlMode.Value("CURRENT"),
}

# ─────────────────────────── 主控制类 ───────────────────────────
class TorqueController:
    """基于关节空间阻抗 + 电流环的 Kinova Gen3 Lite 控制器。"""

    ACTION_TIMEOUT = 20  # 备用（当前版本不再主动移动到 Home）

    # ────────────────────── 初始化 ──────────────────────
    def __init__(self, router, router_rt, urdf_path: str = URDF_PATH):
        # 1) Pinocchio 模型
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()

        # 2) 电机参数：k_t・减速比・效率 → 力矩‑电流常数
        self.torque_constant = np.array([0.0398,0.0398,0.0398,0.0251,0.0251,0.0251])
        self.gear_ratio      = np.array([30,100,30,23,23,23])
        self.gear_efficiency = np.array([0.9,0.65,0.65,1.0,0.9,0.9])
        self.torque_current_constant = (
            self.torque_constant * self.gear_ratio * self.gear_efficiency
        )  # [N·m/A]

        # 3) Kortex 客户端（TCP: router, UDP: router_rt）
        self.device_manager  = DeviceManagerClient(router)
        self.actuator_cfg    = ActuatorConfigClient(router)
        self.base            = BaseClient(router)
        self.base_cyclic     = BaseCyclicClient(router_rt)

        # 4) Router 发送选项（UDP 刷新）
        self.send_opt = RouterClientSendOptions(andForget=False, delay_ms=0, timeout_ms=3)

        # 5) 执行器列表
        handles = self.device_manager.ReadAllDevices().device_handle
        self.actuator_ids = [h.device_identifier for h in handles
                             if h.device_type in (Common_pb2.BIG_ACTUATOR,
                                                 Common_pb2.MEDIUM_ACTUATOR,
                                                 Common_pb2.SMALL_ACTUATOR)]
        self.n = len(self.actuator_ids)

        # 6) 周期命令／反馈原型
        self.base_cmd  = BaseCyclic_pb2.Command()
        self.base_fdbk = BaseCyclic_pb2.Feedback()
        for _ in range(self.n):
            self.base_cmd.actuators.add()
            self.base_fdbk.actuators.add()

        # 7) 线程变量
        self._stop_evt = threading.Event()
        self.loop_th   = None
        self.loop_running = False

    # ────────────────────── 内部工具 ──────────────────────
    def _set_servo_mode(self, mode):
        info = Base_pb2.ServoingModeInformation(servoing_mode=mode)
        self.base.SetServoingMode(info)

    def _call_retry(self, func, retry: int, *args, **kw):
        for _ in range(retry):
            try:
                return func(*args, **kw)
            except Exception:
                continue
        logging.error(f"函数 {func.__name__} 重试 {retry} 次仍失败")
        return None

    def _refresh_fb(self):
        try:
            return self.base_cyclic.Refresh(self.base_cmd, 0, self.send_opt)
        except Exception:
            return None

    # ────────────────────── 初始化周期环 ──────────────────────
    def init_cyclic(self, Ts=0.001, duration: float = 0, print_stats: bool = True):
        """不用回 Home，直接以开机姿态为零点启动阻抗环"""
        if self.loop_running:
            return True

        # 1) 切低级伺服，先获取一帧反馈作为初值
        self._set_servo_mode(Base_pb2.LOW_LEVEL_SERVOING)
        fb = self._refresh_fb()
        if fb is None:
            logging.error("无法获取初始反馈，启动失败")
            return False
        self.base_fdbk = fb

        # 2) 填充命令帧（保持当前位置+当前电流）
        for idx, act in enumerate(fb.actuators):
            self.base_cmd.actuators[idx].flags = BaseCyclic_pb2.ActuatorCommand.Flags.Value("SERVOING")
            self.base_cmd.actuators[idx].position      = act.position
            self.base_cmd.actuators[idx].current_motor = act.current_motor

        # 3) 发送第一帧与执行器改 CURRENT 模式
        self.base_fdbk = self.base_cyclic.Refresh(self.base_cmd, 0, self.send_opt)
        mode_msg = ActuatorConfig_pb2.ControlModeInformation(control_mode=CONTROL_MODES["CURRENT"])
        for aid in self.actuator_ids:
            self._call_retry(self.actuator_cfg.SetControlMode, 3, mode_msg, aid)

        # 4) 记录初始位置 & 预计算梯度
        self._capture_init_state()

        # 5) 线程参数
        self.Ts          = Ts
        self.duration    = duration
        self.print_stats = print_stats
        self.loop_start  = time.time()
        self.last_log    = self.loop_start

        self._stop_evt.clear()
        self.loop_th = threading.Thread(target=self._loop, daemon=True)
        self.loop_th.start()
        self.loop_running = True
        return True

    def _capture_init_state(self):
        fb = self.base_fdbk.actuators
        pos_deg = np.array([a.position for a in fb])
        q0 = np.radians(pos_deg)
        q0 = np.where(q0 > np.pi, q0 - 2*np.pi, q0)
        self.q0 = q0
        self.dq0 = np.zeros_like(q0)
        self.err_int = np.zeros_like(q0)

    # ────────────────────── 1 kHz 循环 ──────────────────────
    def _loop(self):
        logging.info("阻抗循环启动 …")
        while not self._stop_evt.is_set():
            tic = time.time()
            fb = self._refresh_fb()
            if fb is None:
                continue

            # — 解析反馈
            pos_deg = np.array([a.position for a in fb.actuators])
            vel_deg = np.array([a.velocity for a in fb.actuators])
            q  = np.radians(pos_deg)
            dq = np.radians(vel_deg)
            q  = np.where(q > np.pi, q - 2*np.pi, q)

            # — 控制律
            self.err_int += (self.q0 - q) * self.Ts
            g = pin.rnea(self.model, self.data, q, np.zeros_like(q), np.zeros_like(q))
            Kp = np.array([10,10,10,5,2,2])
            Kd = np.ones(6)
            tau = g + Kp*(self.q0 - q) + Kd*(self.dq0 - dq) + 0*self.err_int
            tau = np.clip
            (tau, [-10, -14, -10, -7, -7, -7]
            , [10,14,10,7,7,7])
            I_cmd = tau / self.torque_current_constant

            # — 写命令
            for idx, a in enumerate(self.base_cmd.actuators):
                a.flags = BaseCyclic_pb2.ActuatorCommand.Flags.Value("SERVOING")
                a.position = fb.actuators[idx].position
                a.current_motor = float(I_cmd[idx])
            fid = (self.base_cmd.frame_id + 1) & 0xFFFF
            self.base_cmd.frame_id = fid
            for idx in range(self.n):
                self.base_cmd.actuators[idx].command_id = fid

            self.base_cyclic.Refresh(self.base_cmd, 0, self.send_opt)

            # — 日志
            if self.print_stats and (time.time() - self.last_log >= 1.0):
                self.last_log = time.time()
                logging.info(f"τ_cmd={np.round(tau,2)}  |  I_cmd={np.round(I_cmd,2)}")

            # — 周期休眠
            toc = time.time()
            sleep = self.Ts - (toc - tic)
            if sleep > 0:
                time.sleep(sleep)
            if self.duration and (toc - self.loop_start >= self.duration):
                break

        self.loop_running = False
        logging.info("阻抗循环结束")

    # ────────────────────── 停止并复位 ──────────────────────
    def stop(self):
        if not self.loop_running:
            return
        self._stop_evt.set()
        self.loop_th.join()

        # 执行器改回 POSITION
        mode_msg = ActuatorConfig_pb2.ControlModeInformation(control_mode=CONTROL_MODES["POSITION"])
        for aid in self.actuator_ids:
            self._call_retry(self.actuator_cfg.SetControlMode, 3, mode_msg, aid)
        self._set_servo_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
        logging.info("控制器已安全停机")

    # ────────────────────── 命令行参数 ──────────────────────
    @staticmethod
    def parse_args():
        import argparse
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        import utilities  # Kinova SDK 附带的帮助模块

        parser = argparse.ArgumentParser(description="Kinova Gen3 Lite 阻抗控制（无需回 Home）")
        parser.add_argument("--cyclic_time", type=float, default=0.001, help="控制周期 (s)")
        parser.add_argument("--duration",    type=float, default=0,     help="运行时长，0=无限")
        parser.add_argument("--print_stats", type=lambda x: str(x).lower() not in ["false","0","no"], default=True,
                            help="是否打印状态")
        return utilities.parseConnectionArguments(parser)

# ─────────────────────────── 入口 ───────────────────────────
if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    args = TorqueController.parse_args()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        with utilities.DeviceConnection.createUdpConnection(args) as router_rt:
            ctrl = TorqueController(router, router_rt)
            if not ctrl.init_cyclic(Ts=args.cyclic_time, duration=args.duration, print_stats=args.print_stats):
                sys.exit(1)
            try:
                while ctrl.loop_running:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
            finally:
                ctrl.stop()
