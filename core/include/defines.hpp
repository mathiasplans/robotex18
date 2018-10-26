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
#define SPIN_SEARCH_SPEED     0.5

/**
 * Spinning speed when centering on a ball
 */
#define SPIN_CENTER_SPEED     0.2

/**
 * Moving speed when moving around the arena
 */
#define MOVING_SPEED          35

/**
 * Moving speed when aproaching a ball when throwing
 */
#define MOVING_SPEED_THROW    10

/**
 * Moving speed when orbiting an object
 */
#define ORBIT_SPEED           20.0f

/**
 * Position error of an object. That is how many pixels can an object be out of the intended position.
 */
#define POSITION_ERROR        8

/**
 * Filed of view of the camera, in radians
 */
#define CAMERA_FOV_X          40 /* 40 degrees */

/**
 * How close has the ball have to be for the robot to stop
 */
#define BALL_IN_FRONT         540 /* Height at which the ball is in front of the robot (Y pixels) */

/**
 * How many commands are sent per second
 */
#define COMMAND_RATE          10

/**
 * The delay between the commands
 */
#define COMMAND_DELAY         1000000 / COMMAND_RATE

/**
 * Speed of the thrower and aimer's position
 */
#define THROWER_SPEED         1500
#define AIM_POWER             0

/**
 * The distance from wheel to robot center, in cm
 */

#define WHEEL_D               13

/**
 * The radios of a wheel, in cm
 * 
 * Note: Probably not needed as precise speed is not really needed right now and this acts just as a linear multiplier to all moving speeds
 */
#define WHEEL_R               3.5

/**
 * Wheel speed to mainboard units. Can be calculated using the formula from DigiLabor's omnimotion page. 
 * Value is currently determined experimentally. 
 */

#define MOVING_COEFFICIENT    0.5
