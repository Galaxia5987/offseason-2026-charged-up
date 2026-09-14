package frc.robot.lib

import java.util.stream.Stream

fun <T> Stream<T>.get(index: Long) = this.skip(index + 1).findFirst().get()

fun getDataFromStack(index: Long): StackWalker.StackFrame =
    StackWalker.getInstance().walk { it.get(index) }

fun getFileNameFromStack(index: Long = 2): String =
    getDataFromStack(index).className.substringAfterLast('.')
