#ifndef CONFIGURATION_H
#define CONFIGURATION_H


/// @brief 
class Configuration{
    private:
        /// @brief only instance of Configuraiton.
        static Configuration* instance;
        /// @brief 3D array [CAN][MOTOR_ID][STATE] which stores the kinematics on each state based off of can bus and motor id.
        float kinematics[2][NUM_MOTORS][STATE_LEN];
        /// @brief 2D array which stores the control gains for each motor in format [p_p, i_p, d_p, f_p, p_v, i_v, d_v, f_v, p_a....] this format can change depending on control type.
        float controller_weights[NUM_MOTORS][WEIGHTS_LEN];
        /// @brief 1D array which stores the type of control each motor uses.
        int controller_types[NUM_MOTORS];
    
    public:
        /// @brief get the instance of singleton, if there is no instance then instantiate and return.
        /// @return instance of Configuration singleton.
        static Configuration* get_instance(){
            if(instance == nullptr){
                instance = new Configuration();
                return instance;
            }
            return instance;
        };

        /// @brief get 3D kinematics array.
        /// @param _kinematics array that the kinematics array will be copied over to using memcpy.
        void get_kinematics(float _kinematics[2][NUM_MOTORS][STATE_LEN]);

        /// @brief get 2d controller_weights array.
        /// @param _controller_weights  array that controller_weights will be copied over to using memcpy.
        void get_controller_weights(float _controller_weights[NUM_MOTORS][WEIGHTS_LEN]);

        /// @brief get 1D controller_types array.
        /// @param _controller_types array that controller_types will be copied over to using memcpy.
        void get_controller_types(int _controller_types[NUM_MOTORS]);

        /// @brief set 3D kinematics array.
        /// @param _kinematics new 3D array that will be copied over using memcpy.
        /// @param size size in bytes of new array.
        void set_kinematics(float _kinematics[2][NUM_MOTORS][STATE_LEN], int size);

        /// @brief set 2D controller_weights array.
        /// @param _controller_weights new 2D array that will be copied over using memcpy.
        /// @param size size in bytes of new array.
        void set_controller_weights(float _controller_weights[NUM_MOTORS][WEIGHTS_LEN], int size);

        /// @brief set 1D controller_types array.
        /// @param _controller_types new 1D array that will be copied over using memcpy.
        /// @param size size in bytes of new array.
        void set_controller_types(int _controller_types[NUM_MOTORS], int size);
}
#endif