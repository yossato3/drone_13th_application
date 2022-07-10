"""
ドローンソフトウェアエンジニア養成塾13期
Applicationコース
講座修了試験

ドローンが着地している状態から実行してください。
ドローンがリサージュ図形的な何かの軌跡に沿って飛行し、RTLで着地したら終了します。

パラーメータを変更する際は、合わせ込みをしないとドローンがあらぬ方向にすっ飛んでいきます。
"""

import time
import math
from dronekit import connect, Command
from pymavlink import mavutil

CONNECT_STRING = '127.0.0.1:14550'  # 接続文字列

ALTITUDE = 10                       # 飛行高度[m]
FLIGHT_AREA = 60                    # 飛行範囲[m]

ROUND_TIME = 90                     # 一周するおよその時間[sec]
COMMAND_PERIOD = 1.0                # コマンド発行間隔[sec]

LISSAJOUS_X = 2                     # リサージュ X方向振動数
LISSAJOUS_Y = 3                     # リサージュ Y方向振動数

class Drone:
    """
    ドローンクラス
    """
    EARTH_LADIUS = 6371000          # 地球の半径[m]

    def __init__(self, connect_string):
        """
        初期化処理
        """
        self._vehicle = None
        self._origin_lat = 0
        self._origin_lon = 0
        self._origin_x = None
        self._origin_y = None
        self._ready = False
        # SITLと接続
        self._connect(connect_string)
    
    def _connect(self, connect_string):
        """
        ドローンに接続
        """
        self._vehicle = connect(connect_string, wait_ready=True, timeout=60)

    def Initialize(self):
        """
        原点取得処理
        """
        if self._vehicle is None:
            raise Exception

        while (self._vehicle.location.local_frame.north is None) or (self._vehicle.location.local_frame.east is None):
            time.sleep(0.5)

        self._origin_x = self._vehicle.location.local_frame.north
        self._origin_y = self._vehicle.location.local_frame.east
        self._origin_lat = self._vehicle.location.global_frame.lat
        self._origin_lon = self._vehicle.location.global_frame.lon
        self._ready = True

    def close(self):
        """
        終了処理
        """
        self._ready = False
        if self._vehicle is not None:
            self._vehicle.close()

    def return_to_launch(self):
        """
        RTL
        """
        if self._vehicle is None:
            raise Exception

        self._vehicle.wait_for_mode("RTL")
        while self._vehicle.armed:
            time.sleep(0.5)

    @property
    def armed(self):
        if self._vehicle is None:
            return False
        else:
            return self._vehicle.armed
        
    def _get_lissajous_position(self, l_time):
        """
        指定時間のドローンの目標位置を返す
        ホームロケーション基準の[m]単位
        """
        y = FLIGHT_AREA / 2 * math.sin(LISSAJOUS_X * 2 * math.pi * l_time / ROUND_TIME)
        x = FLIGHT_AREA / 2 * math.sin(LISSAJOUS_Y * 2 * math.pi * l_time / ROUND_TIME)
        return x, y

    def _latlon_to_xy(self, lat, lon):
        """
        未使用
        緯度経度から原点基準の[m]位置を計算
        地球は真円。細かいことは気にしない。
        """
        if not self._ready:
            raise Exception

        x = (lat - self._origin_lat) / 360.0 * (2 * math.pi * Drone.EARTH_LADIUS)
        y = math.cos(math.radians(self._origin_lat)) * ((lon - self._origin_lon) / 360.0 * (2 * math.pi * Drone.EARTH_LADIUS))
        return x, y

    def _xy_to_latlon(self, x, y):
        """
        原点基準の[m]位置から緯度経度を計算
        ミッション作成に使用
        """
        if not self._ready:
            raise Exception

        lat = self._origin_lat + x / (2 * math.pi * self.EARTH_LADIUS) * 360
        lon = self._origin_lon + y / (math.cos(math.radians(lat)) * (2 * math.pi * Drone.EARTH_LADIUS)) * 360
        return lat, lon

    def create_mission(self):
        """
        ミッション作成
        実際の飛行との比較用
        """
        if not self._ready:
            raise Exception

        commands = self._vehicle.commands
        commands.download()
        commands.wait_ready()
        commands.clear()

        for t in range(ROUND_TIME):
            x, y = self._get_lissajous_position(t + 1)
            lat, lon = self._xy_to_latlon(x, y)
            print("  WAYPOINT lat={}, lon={}".format(lat,lon))
            command = Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, ALTITUDE)
            commands.add(command)

        commands.upload()

    def arming(self):
        """
        モードをGUIDEDにしてアーミング
        """
        if not self._ready:
            raise Exception

        self._vehicle.wait_for_armable()
        self._vehicle.wait_for_mode("GUIDED")
        self._vehicle.arm()

    def takeoff(self):
        """
        離陸
        """
        self._vehicle.wait_simple_takeoff(ALTITUDE, timeout=20)

    def exec(self):
        """
        飛行
        次の目標に向かっていくだけ
        ちゃんとやるなら色々難しいことを考える
        """
        if not self._ready:
            raise Exception

        current_time = 0
        current_x = 0
        current_y = 0
        first_time = True
        count = 0

        while True:
            # 前回位置
            pre_x = current_x
            pre_y = current_y
            # 現在位置
            current_x = self._vehicle.location.local_frame.north - self._origin_x
            current_y = self._vehicle.location.local_frame.east - self._origin_y

            if first_time:
                # 初回
                current_time = current_time + COMMAND_PERIOD
                first_time = False
            else:
                # 二回目以降
                
                # 実際の移動距離
                actual_dist = math.sqrt((current_x - pre_x) ** 2 + (current_y - pre_y) ** 2)
                # 目標とした移動距離(前回設定した速度より)
                target_dist = math.sqrt(velocity_x ** 2 + velocity_y ** 2) * COMMAND_PERIOD

                if actual_dist > target_dist:
                    # 目標より飛び過ぎたときは補正しない(あらぬ方向に行ってしまうことがある)
                    current_time = current_time + COMMAND_PERIOD
                else:
                    # 目標に満たなかった時は次のポイントを補正する
                    current_time = current_time + actual_dist / target_dist * COMMAND_PERIOD
            
            # 一周したら終了
            if current_time >= ROUND_TIME:
                break

            # 目標地点を計算
            target_x, target_y = self._get_lissajous_position(current_time)
            # 速度を計算。飛び過ぎないように少し遅めにする。
            velocity_x = (target_x - current_x) / COMMAND_PERIOD * 0.8
            velocity_y = (target_y - current_y) / COMMAND_PERIOD * 0.8

            # print
            count = count + 1
            print("  {}/{} 現在位置({:.2f},{:.2f}) 目標位置({:.2f},{:.2f}) 設定速度({:.2f},{:.2f})".format(count,int(ROUND_TIME/COMMAND_PERIOD),current_x,current_y,target_x,target_y,velocity_x,velocity_y))

            # mavlinkメッセージ作成
            msg = self._vehicle.message_factory.set_position_target_local_ned_encode(
                0,                  # time_boot_ms (not used)
                0, 0,               # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                0b0000111111000111, # type_mask
                0, 0, 0,            # x, y, z positions (not used)
                velocity_x, velocity_y, 0,    # x, y, z velocity in m/s
                0, 0, 0,            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)               # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            # mavlinkメッセージ送信
            self._vehicle.send_mavlink(msg)

            # 次のメッセージまで待ち
            time.sleep(COMMAND_PERIOD)

if __name__ == '__main__':
    drone = None
    try:
        print("接続中...")
        drone = Drone(CONNECT_STRING)
        print("接続完了")

        print("初期化中...")
        drone.Initialize()
        print("初期化完了")

        # 軌跡確認用
        #print("ミッション作成中...")
        #drone.create_mission()
        #print("ミッション作成完了")

        print("アーム中...")
        drone.arming()
        print("アーム完了")

        print("離陸中...")
        drone.takeoff()
        print("離陸完了")

        print("飛行中...")
        drone.exec()
        print("飛行完了")

    except:
        print("\n\n例外処理")

    finally:
        if drone is not None:
            if drone.armed:
                print("RTL...")
                drone.return_to_launch()
                print("RTL完了")

            drone.close()

    print("終了")

