#include "walkinggait/WalkingCycle.hpp"

WalkingCycle::WalkingCycle()
{
    IsStop = false;
    
    Sample_points_quater = 0;
}

WalkingCycle::~WalkingCycle()
{

}

void WalkingCycle::walkingKindFunction(int walking_mode)
{
    switch(walking_mode)
    {
        case 0://Single Step
        case 2://LC Up
        case 3://LC Down
        case 4://Long Jump
        case 5://Single Wood
        case 6://Single Third
            singleStepFunction(walking_mode);
            break;
        case 1://Continuous
        case 7://Continuous Second
        case 8://Continuous Third
            continuousStepFunction();
            break;
        default:
            break;
    }
}

void WalkingCycle::singleStepFunction(int walking_mode)
{
    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);

    if(walking_mode == 0 || walking_mode == 4 || walking_mode == 5 || walking_mode == 6)
    {
        singleStepWalkingProcess();
    }
    else
    {
        LCWalkingProcess();
    }
}

void WalkingCycle::singleStepWalkingProcess()
{
    parameterinfo->counter++;
    switch(parameterinfo->complan.walking_state)
    {
        case StartStep:
            parameterinfo->complan.walking_stop = false;

            slopeCounter_ = 0;
            forwardValue_ = 0.0;
            forwardCounter_ = 0.0;

            if(!parameterinfo->IsParametersLoad)
            {
                parameterinfo->complan.isfirststep = true;
                parameterinfo->IsParametersLoad = true;
                parameterinfo->complan.islaststep = false;
                parameterinfo->complan.isLfootfirst = false;

                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->THETAUpdate = parameterinfo->THETA;

                Lockrange_tmp = parameterinfo->parameters.OSC_LockRange;
                COM_Y_tmp = parameterinfo->parameters.Y_Swing_Range;
                parameterinfo->parameters.OSC_LockRange  = parameterinfo->parameters.OSC_LockRange;
                parameterinfo->parameters.Y_Swing_Range = parameterinfo->parameters.Y_Swing_Range + 0.5;
            }

            if(parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T2)
            {
                parameterinfo->parameters.Y_Swing_Range = COM_Y_tmp;
                parameterinfo->parameters.OSC_LockRange = Lockrange_tmp;
                parameterinfo->complan.isfirststep = false;
            }

            if(parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T * 3/4) // The first Step
            {
                parameterinfo->complan.walking_state = FirstStep;
                parameterinfo->IsParametersLoad = false;
                slopeCounter_++;
                forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                forwardCounter_ += forwardValue_;
            }
            else
            {
                parameterinfo->XUpdate = forwardValue_;
            }
            break;
        case FirstStep:
            parameterinfo->complan.isfirststep = false;

            if(parameterinfo->X > forwardCounter_)
            {
                parameterinfo->XUpdate = forwardValue_;

                if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 1/4)
                {
                    parameterinfo->complan.walking_state = Repeat;
                    parameterinfo->IsParametersLoad = false;
                    slopeCounter_++;
                    forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                    forwardCounter_ += forwardValue_;
                }
            }
            else
            {
                parameterinfo->XUpdate = parameterinfo->X;

                if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 1/4)
                {
                    parameterinfo->complan.walking_state = StopStep;
                    parameterinfo->IsParametersLoad = false;
                    parameterinfo->complan.islaststep = true;//rotate compensation
                }
            }
            break;
        case Repeat:
            if(parameterinfo->X > forwardCounter_)
            {
                parameterinfo->XUpdate = forwardValue_;

                if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 1/4 || parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 3/4)
                {
                    parameterinfo->complan.walking_state = Repeat;
                    parameterinfo->IsParametersLoad = false;

                    if(forwardValue_ >= WALK_MAX_DISTANCE)
                    {
                        forwardValue_ = WALK_MAX_DISTANCE;
                    }
                    else
                    {
                        slopeCounter_++;
                        forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                    }

                    forwardCounter_ += forwardValue_;
                }
            }
            else
            {
                parameterinfo->THETAUpdate /= 1000;
                parameterinfo->YUpdate /= 1000;

                if(parameterinfo->X == forwardCounter_)
                {
                    parameterinfo->XUpdate = forwardValue_;

                }
                else
                {
                    parameterinfo->XUpdate = parameterinfo->X - (forwardCounter_ - forwardValue_);
                }

                if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 1/4 || parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 3/4)
                {
                    parameterinfo->complan.walking_state = StopStep;
                    parameterinfo->IsParametersLoad = false;
                    parameterinfo->complan.islaststep = true;//rotate compensation
                }
            }
            break;
        case StopStep:
            parameterinfo->XUpdate = forwardCounter_ = 0;

            if(Sample_points_quater == parameterinfo->parameters.Sample_Time/4 - 1)
            {
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->complan.walking_stop = true;
                parameterinfo->IsParametersLoad = false;
                parameterinfo->WalkFlag = false;
                parameterinfo->FpgaFlag = false;
                Sample_points_quater = 0;
                parameterinfo->complan.sample_point_ = 0;
            }
            else
            {
                Sample_points_quater++;
            }
            break;
        default:
            parameterinfo->complan.walking_state = StartStep;
            break;
    }
    parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
}

void WalkingCycle::LCWalkingProcess()
{
    switch(parameterinfo->complan.walking_state)
    {
        case StartStep:
            parameterinfo->complan.walking_stop = false;

            slopeCounter_ = 0;
            forwardValue_ = 0.0;
            forwardCounter_ = 0.0;

            if(!parameterinfo->IsParametersLoad)
            {
                parameterinfo->complan.isfirststep = true;
                parameterinfo->IsParametersLoad = true;
                parameterinfo->complan.islaststep = false;
                parameterinfo->complan.isLfootfirst = false;

                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;

                Lockrange_tmp = parameterinfo->parameters.OSC_LockRange;
                COM_Y_tmp = parameterinfo->parameters.Y_Swing_Range;
                parameterinfo->parameters.OSC_LockRange  = parameterinfo->parameters.OSC_LockRange;
                parameterinfo->parameters.Y_Swing_Range = parameterinfo->parameters.Y_Swing_Range;

            }

            if(parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T / 4) 
            {
                parameterinfo->complan.walking_state = FirstStep;
                parameterinfo->IsParametersLoad = false;
                parameterinfo->complan.isfirststep = false;
                parameterinfo->parameters.Y_Swing_Range = COM_Y_tmp;
                parameterinfo->complan.lift_lock_x = 0;
            }
            else
            {
                parameterinfo->XUpdate = 0;
            }
            break;
        case FirstStep:
            parameterinfo->complan.isfirststep = false;

            parameterinfo->XUpdate = parameterinfo->X;

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 3/4)
            {
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->IsParametersLoad = false;
                parameterinfo->complan.islaststep = true;//rotate compensation
            } 
            break;
        case StopStep:
            parameterinfo->X = 0;
            parameterinfo->XUpdate = 0;

            if(Sample_points_quater == parameterinfo->parameters.Sample_Time * 1/4 - 1)
            {
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->complan.walking_stop = true;
                parameterinfo->IsParametersLoad = false;
                parameterinfo->FpgaFlag = false;
                Sample_points_quater = 0;
                parameterinfo->complan.sample_point_ = 0;
            }
            else
            {
                Sample_points_quater++;
            }
            break;
        default:
            parameterinfo->complan.walking_state = StartStep;
            break;
    }

    parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
    parameterinfo->counter++;
}

void WalkingCycle::continuousStepFunction()
{
    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    continuousWalkingProcess();
}

void WalkingCycle::continuousWalkingProcess()
{
    parameterinfo->counter++;
    switch(parameterinfo->complan.walking_state)
    {
        case StartStep:
            parameterinfo->complan.walking_stop = false;
            parameterinfo->complan.isfirststep = true;
            parameterinfo->Isfrist_right_shift = true;

            if(!parameterinfo->IsParametersLoad) 
            {
                parameterinfo->IsParametersLoad = true;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
            }

            if (parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T * 1/4)
            {
                parameterinfo->complan.walking_state = MarkTimeStep;
                parameterinfo->IsParametersLoad = false;
            }
            else
            {
                parameterinfo->XUpdate = 0;
            }
            break;
        case MarkTimeStep:
            if(!parameterinfo->IsParametersLoad)
            {
                parameterinfo->IsParametersLoad = true;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
            }

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 1/2)
            {
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;
            }

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 3/4)
            {
                parameterinfo->complan.isfirststep = false;
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;

                if(parameterinfo->XUpdate > 0)
                {
                    parameterinfo->complan.walking_state = ForwardStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else if(parameterinfo->XUpdate < 0)
                {
                    parameterinfo->complan.walking_state = BackwardStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else
                {
                    parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
                }
            }

            if(parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T)
                parameterinfo->Isfrist_right_shift = false;
            break;
        case ForwardStep:
            if(!parameterinfo->IsParametersLoad)
            {
                parameterinfo->IsParametersLoad = true;
            }

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 1/2)
            {
                if(parameterinfo->parameters.OSC_LockRange < 0)
                {
                    parameterinfo->parameters.OSC_LockRange = 0;
                }

                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;
            }

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T ==  parameterinfo->parameters.Period_T * 3/4)
            {
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;

                if(parameterinfo->XUpdate == 0) 
                {
                    parameterinfo->complan.walking_state = MarkTimeStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else 
                {
                    parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
                }
            }

            if(parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T)
                parameterinfo->Isfrist_right_shift = false;
            break;
        case BackwardStep:           
            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 1/2)
            {
                if(parameterinfo->parameters.OSC_LockRange < 0)
                {
                    parameterinfo->parameters.OSC_LockRange = 0;
                }

                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;
            }

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 3/4)
            {
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;

                if(parameterinfo->XUpdate == 0) 
                {
                    parameterinfo->complan.walking_state = MarkTimeStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else 
                {
                    parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
                }
            }

            if(parameterinfo->complan.time_point_  == parameterinfo->parameters.Period_T)
                parameterinfo->Isfrist_right_shift = false;
            break;
        case StopStep:
            if(!parameterinfo->IsParametersLoad) 
            {
                parameterinfo->IsParametersLoad = true;
            }

            if(Sample_points_quater > (parameterinfo->parameters.Sample_Time * 3/4) && IsStop)
            {
                parameterinfo->XUpdate = 0;
            }
            else if(parameterinfo->counter <= parameterinfo->parameters.Sample_Time * 3/4)
            {
                parameterinfo->XUpdate = 0;
            }
            else
            {
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THETAUpdate = parameterinfo->THETA;

                if(IsStop && (parameterinfo->Y > 0.0) && (Sample_points_quater >= parameterinfo->parameters.Sample_Time * 1/2))
                {
                    parameterinfo->YUpdate = 0;
                }
                Sample_points_quater++;
            }

            if((parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T) == 0 && IsStop)
            {
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->complan.walking_stop = true;
                parameterinfo->IsParametersLoad = false;
                parameterinfo->FpgaFlag = false;
                parameterinfo->complan.sample_point_ = 0;

                IsStop = false;
                Sample_points_quater = 0;
            }
            else if((parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T) == 0 && !IsStop)
            {
                IsStop = true;
                Sample_points_quater = 0;
            }
            break;
    }
}
