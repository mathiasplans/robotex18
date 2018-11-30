/**
 * Frame dimensions
 * TODO: Get camera node to use these for configuration
 * Note: Maybe use a camera driver such as http://wiki.ros.org/libuvc_camera, then camera conf is done through a launch file. Can use these there.
 */
#define FRAME_HEIGHT          640
#define FRAME_WIDTH           480

/**
 * Spinning speed when searching for a ball/basket
 */
#define SPIN_SEARCH_SPEED     1.6

/**
 * Spinning speed when centering on a ball
 */
#define SPIN_CENTER_SPEED     0.2

/**
 * Moving speed when moving around the arena
 */
#define MOVING_SPEED          150

/**
 * Moving speed when aproaching a ball when throwing
 */
#define MOVING_SPEED_THROW    31

/**
 * Moving speed when orbiting an object
 */
#define ORBIT_SPEED           65

/**
 * Position error of an object. That is how many pixels can an object be out of the intended position.
 */
#define POSITION_ERROR        6

/**
 * Filed of view of the camera, in radians
 */
#define CAMERA_FOV_X          40 /* 40 degrees */

/**
 * How close has the ball have to be for the robot to stop
 */
#define BALL_IN_FRONT         FRAME_HEIGHT / 1.57 /* Height at which the ball is in front of the robot (Y pixels) */

/**
 * How many commands are sent per second
 */
#define COMMAND_RATE          20

/**
 * The delay between the commands
 */
#define COMMAND_DELAY         1000000 / COMMAND_RATE

/**
 * Speed of the thrower and aimer's position
 */
#define THROWER_SPEED         1500
#define AIM_POWER             1001

/**
 * The distance from wheel to robot center, in cm
 */

#define WHEEL_D               13

/**
 * The radius of a wheel, in cm
 *
 * Note: Probably not needed as precise speed is not really needed right now and this acts just as a linear multiplier to all moving speeds
 */
#define WHEEL_R               3.5

/**
 * Wheel speed to mainboard units. Can be calculated using the formula from DigiLabor's omnimotion page.
 * Value is currently determined experimentally.
 */

#define MOVING_COEFFICIENT    0.5

/**
 * When the robot starts to throw a ball, the ball will be out of frame.
 * Normally the robot would reset and start searchig for a ball, but we
 * want it to continue for a specific time so that
 * the thwow will be completed instead.
 *
 * In seconds
 */
#define THROW_TIME            3
