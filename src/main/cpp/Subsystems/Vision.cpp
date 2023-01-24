#include "Subsystems/Vision.h"
bool Vision::Init () {
    ResetSubsystem();
    return true;

}


        void Periodic() {
            
        }
        void SimulationPeriodic() {

        }

        bool GetConeError (double *error) {
            *error = interface_->cone_error;
            return true;
        }
        bool GetConeDistance (double *cone){
            *cone = interface_->cone_distance;
            return true;
        }
        bool GetCubeDistance (double *cube){
            *cube = interface_->cube_distance;
            return true;
        }

        bool GetCubeAngle (double *angle){
            *angle = interface_->angle_cube;
        }
        bool ResetSubsystem (){
            interface_->resetsubsystem_ = true;
            return true;
        }

        
       
