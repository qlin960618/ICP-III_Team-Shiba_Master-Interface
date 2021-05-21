# Motion Capture Master Interface

## Class: CameraReader

> This class object will handle operation of each camera. That mean for a three
 cameras motion capture setup, the main program will instantiate three instances
 of the Class. The computation loop should be non blocking. That mean program utilizing the Threading
 library for parallel execution.

### Public function definition
> if not specifically specified, function should be non-blocking

  1. def \_Constructor_ ( cameraIndex )

    - cameraIndex:
      - camera index or device path depending on the requirement of
      openCV video capture handling

  2. def begin_capture()

    - return: bool -- success
      - If the initialization is successful return "True" else is "False"  

  3. def get_frame_size ()

    - return: int, int -- x_size, y_size
      - x_size, y_size of the camera Frame

  4. def frame_ready ( int )

    - int -- timeout
      - timeout value of how long the function should wait: -1=Inf, 0=no wait
      time
    - return: bool -- ready
      - return true if ball is in frame and new data/frame is ready for extracting.
      else return false when function timed out. (This is a blocking function)

  5. def get_ball_position (int)

    - int -- ballID
      - The id of the ball to locate. the ID can be hardcoded with color. Ex.
      Green=1, Red=2
    - return: bool, int, int -- ball_present, x, y
      - Boolean of if the ball with given ID is in frame
      - x, y pixel of where the ball with given ID is located in the camera.
      if ball is not present, function should return the last known position of
      the ball.

  6. def get_ball_dq_origin (int)

    - int -- ballID
      - same as above
    - return: bool, DQ -- ball_present, ball_vec
      - Boolean of if the ball with given ID is in frame
      - Unit DQ of where the ball is pointing from Camera origin Reference Frame.
      Processing of this function should also account for camera lens optical
      distortion. Question of how, should be handled internally as cameras model
      used is the same. Behavior under error condition should be same as previous.   

  7. def get_ball_dq_pov (int, DQ)

    - int -- ballID
      - same as above
    - DQ -- cameraDQ
      - The DQ representing the camera position and facing direction
    - return: bool, DQ -- ball_present, ball_vec
      - Boolean of if the ball with given ID is in frame
      - Unit DQ representing a line in space of where the ball with give ID could
      be located.

  8. def \_Destructor_()

    - graceful handling of the exiting process and resource deallocation.
