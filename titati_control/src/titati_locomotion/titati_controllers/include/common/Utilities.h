#include <iostream>

template < typename T > T WrapRange( const T& target, const T& min, const T& max ) {
    T result = target;
    assert( min <= max );
    if ( result < min ) {
        result = min;
    }
    if ( max < result ) {
        result = max;
    }
    return result;
}

template < typename T >
T ApplyVelocityMeetAccelationLimit( const T& current_velocity, const T& target_velocity, const T& velocity_minium, const T& velocity_maxium, const T& accelation_minium, const T& accelation_maxium,
                                    double dt ) {
    // apply deadband
    T valid_target_vel = WrapRange( target_velocity, velocity_minium, velocity_maxium );
    if ( std::fabs( current_velocity - valid_target_vel ) < 0.02 ) {
        return valid_target_vel;
    }

    T min_vel_acc_limited = current_velocity + accelation_minium * dt;
    T max_vel_acc_limited = current_velocity + accelation_maxium * dt;

    T result_vel = target_velocity;
    // apply min limit
    if ( min_vel_acc_limited <= max_vel_acc_limited )
        result_vel = WrapRange( result_vel, min_vel_acc_limited, max_vel_acc_limited );
    else
        result_vel = WrapRange( result_vel, max_vel_acc_limited, min_vel_acc_limited );
    result_vel = WrapRange( result_vel, velocity_minium, velocity_maxium );

    return result_vel;
}
