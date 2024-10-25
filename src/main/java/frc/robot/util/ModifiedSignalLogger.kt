package frc.robot.util

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.hardware.ParentDevice
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import java.util.function.Consumer

object ModifiedSignalLogger : SignalLogger() {
    fun logState(): Consumer<SysIdRoutineLog.State> {
        start() // Start logging if we get the consumer, so we have some data before the start of the motion
        return Consumer { state: SysIdRoutineLog.State -> writeString("State", state.toString()) }
    }

    fun registerAsSysIdLog(device: ParentDevice) {
        writeInteger("SysId Logged Device", device.getDeviceHash(), "")
    }
}
