// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PS5Controller
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.event.BooleanEvent
import edu.wpi.first.wpilibj.event.EventLoop
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BiFunction
import java.util.function.BooleanSupplier

/**
 * A version of [XboxController] with [Trigger] factories for command-based.
 *
 * @see XboxController
 */
class CommandXboxPS5Controller(port: Int) : CommandGenericHID(port) {
    private var m_hidx: XboxController? = null
    private var m_hidp: PS5Controller? = null
    private var xbox: Boolean? = null

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    init {
        if (DriverStation.getJoystickName(port).contains("Wireless")) {
            xbox = false
            m_hidp = PS5Controller(port)
        } else {
            xbox = true
            m_hidx = XboxController(port)
        }
    }

    /**
     * Get the underlying GenericHID object.
     *
     * @return the wrapped GenericHID object
     */
    override fun getHID(): XboxController {
        return m_hidx!!
    }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @return an event instance representing the left bumper's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .leftBumper
     */
    @JvmOverloads
    fun leftBumper(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.leftBumper(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.L1(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @return an event instance representing the right bumper's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .rightBumper
     */
    @JvmOverloads
    fun rightBumper(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.rightBumper(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.R1(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal attached to the
     * given loop.
     */
    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @return an event instance representing the left stick button's digital signal attached to the
     * [default scheduler button loop][CommandScheduler.getDefaultButtonLoop].
     * @see .leftStick
     */
    @JvmOverloads
    fun leftStick(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.leftStick(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.L3(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital signal attached to the
     * given loop.
     */
    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @return an event instance representing the right stick button's digital signal attached to the
     * [default scheduler button loop][CommandScheduler.getDefaultButtonLoop].
     * @see .rightStick
     */
    @JvmOverloads
    fun rightStick(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.rightStick(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.R3(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @return an event instance representing the A button's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .a
     */
    @JvmOverloads
    fun a(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.a(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.cross(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @return an event instance representing the B button's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .b
     */
    @JvmOverloads
    fun b(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.b(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.circle(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @return an event instance representing the X button's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .x
     */
    @JvmOverloads
    fun x(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.x(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.square(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @return an event instance representing the Y button's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .y
     */
    @JvmOverloads
    fun y(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.y(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.triangle(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @return an event instance representing the start button's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .start
     */
    @JvmOverloads
    fun start(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.start(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.options(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given
     * loop.
     */
    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @return an event instance representing the back button's digital signal attached to the [     ][CommandScheduler.getDefaultButtonLoop].
     * @see .back
     */
    @JvmOverloads
    fun back(loop: EventLoop? = CommandScheduler.getInstance().defaultButtonLoop): Trigger {
        return if (xbox!!) {
            m_hidx!!.back(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            m_hidp!!.create(loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        }
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than `threshold`.
     *
     * @param threshold the minimum axis value for the returned [Trigger] to be true. This value
     * should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     * threshold, attached to the given event loop
     */
    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than `threshold`.
     *
     * @param threshold the minimum axis value for the returned [Trigger] to be true. This value
     * should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided
     * threshold, attached to the [default scheduler][CommandScheduler.getDefaultButtonLoop].
     */
    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to
     * the [default scheduler button loop][CommandScheduler.getDefaultButtonLoop].
     */
    @JvmOverloads
    fun leftTrigger(
        threshold: Double = 0.5,
        loop: EventLoop = CommandScheduler.getInstance().defaultButtonLoop,
    ): Trigger {
        return if (xbox!!) {
            m_hidx!!.leftTrigger(threshold, loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            BooleanEvent(loop) { m_hidp!!.l2Axis > threshold }.castTo<Trigger>(
                BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                    Trigger(
                        loop,
                        condition,
                    )
                },
            )
        }
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than `threshold`.
     *
     * @param threshold the minimum axis value for the returned [Trigger] to be true. This value
     * should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
     * threshold, attached to the given event loop
     */
    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than `threshold`.
     *
     * @param threshold the minimum axis value for the returned [Trigger] to be true. This value
     * should be in the range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided
     * threshold, attached to the [default scheduler][CommandScheduler.getDefaultButtonLoop].
     */
    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to
     * the [default scheduler button loop][CommandScheduler.getDefaultButtonLoop].
     */
    @JvmOverloads
    fun rightTrigger(
        threshold: Double = 0.5,
        loop: EventLoop = CommandScheduler.getInstance().defaultButtonLoop,
    ): Trigger {
        return if (xbox!!) {
            m_hidx!!.rightTrigger(threshold, loop)
                .castTo<Trigger>(
                    BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                        Trigger(
                            loop,
                            condition,
                        )
                    },
                )
        } else {
            BooleanEvent(loop) { m_hidp!!.r2Axis > threshold }.castTo<Trigger>(
                BiFunction<EventLoop, BooleanSupplier, Trigger> { loop: EventLoop?, condition: BooleanSupplier? ->
                    Trigger(
                        loop,
                        condition,
                    )
                },
            )
        }
    }

    val leftX: Double
        /**
         * Get the X axis value of left side of the controller.
         *
         * @return The axis value.
         */
        get() = if (xbox!!) {
            m_hidx!!.leftX
        } else {
            m_hidp!!.leftX
        }

    val rightX: Double
        /**
         * Get the X axis value of right side of the controller.
         *
         * @return The axis value.
         */
        get() {
            return if (xbox!!) {
                m_hidx!!.rightX
            } else {
                m_hidp!!.rightX
            }
        }

    val leftY: Double
        /**
         * Get the Y axis value of left side of the controller.
         *
         * @return The axis value.
         */
        get() {
            return if (xbox!!) {
                m_hidx!!.leftY
            } else {
                m_hidp!!.leftY
            }
        }

    val rightY: Double
        /**
         * Get the Y axis value of right side of the controller.
         *
         * @return The axis value.
         */
        get() {
            return if (xbox!!) {
                m_hidx!!.rightY
            } else {
                m_hidp!!.rightY
            }
        }

    val leftTriggerAxis: Double
        /**
         * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        get() {
            return if (xbox!!) {
                m_hidx!!.leftTriggerAxis
            } else {
                m_hidp!!.l2Axis
            }
        }

    val rightTriggerAxis: Double
        /**
         * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
         * range of [0, 1] as opposed to the usual [-1, 1].
         *
         * @return The axis value.
         */
        get() {
            return if (xbox!!) {
                m_hidx!!.rightTriggerAxis
            } else {
                m_hidp!!.r2Axis
            }
        }
}
