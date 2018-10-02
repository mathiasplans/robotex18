
/**
 * Spinning speed when searching for a ball/basket
 */
#define SPIN_SEARCH_SPEED     5

/**
 * Spinning speed when centering on a ball
 */
#define SPIN_CENTER_SPEED     1

/**
 * Moving speed when moving around the arena
 */
#define MOVING_SPEED          15

/**
 * Moving speed when aproaching a ball when throwing
 */
#define MOVING_SPEED_THROW    1

/**
 * Moving speed when orbiting an object
 */
#define ORBIT_SPEED           1

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
 * Speed of the thrower
 */
#define THROWER_SPEED         1500

/**
 * The diameter of a wheel, in cm
 */
#define WHEEL_D               6.8

/**
 * The radios of a wheel, in cm
 */
#define WHEEL_R               WHEEL_D / 2
