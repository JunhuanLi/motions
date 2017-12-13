#ifndef MOTIONENUMS_H_
#define MOTIONENUMS_H_

namespace motion
{
    enum ERotateOrientation
    {
        ROTATECCW = 0,   /**< rotate counterclockwise */
        ROTATECW         /**< rotate clockwise */
    };

    enum ESteeringStateType
    {
        NONE = 0,   /**< initial state */
        EXECUTING,  /**< the current steering order is executing */
        SUCCESS,    /**< the steering order is successed */
        FAILED      /**< the steering order is failed */
    };

}

#endif //MOTIONENUMS_H_

