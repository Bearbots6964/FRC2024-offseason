package frc.robot.util

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import java.util.function.Consumer

object ModifiedSignalLogger : SignalLogger() {
    fun logState(): Consumer<SysIdRoutineLog.State> {
        // Start logging if we get the consumer, so we have some data
        // before the start of the motion
        start()
        
        return Consumer { state: SysIdRoutineLog.State -> writeString("State", state.toString()) }
    }
}
