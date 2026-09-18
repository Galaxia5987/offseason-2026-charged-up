package frc.robot.lib

class Flipper<T>(supplier: (isRed: Boolean) -> T){
    val red: T = supplier(true)
    val blue: T = supplier(false)

    fun get(): T = if(IS_RED) red else blue
}

fun <T, K> T.flipper() where T : (isRed: Boolean) -> K = Flipper(this)