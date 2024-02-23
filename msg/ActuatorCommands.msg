# mecha_control/msg/ActuatorCommands.msg

float64[] motor_positions    # モータの位置や角度を制御するための値
bool[] motor_expand          # モータとリミットスイッチによる0/1制御のオン指令
bool[] cylinder_states       # エアシリンダのオン/オフ状態