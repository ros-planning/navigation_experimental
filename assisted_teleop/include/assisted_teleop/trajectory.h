/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Sachin Chitta */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <sstream>
#include <map>

#include "ros/console.h"

namespace trajectory
{
  /*! \class 
    \brief The trajectory class specifies general n-DOF trajectories and allows the user to sample trajectories based on a number of interpolation schemes. A trajectory is composed of a set of trajectory points. Interpolation between trajectory points uses the interpolation scheme set by the user (default is linear). The TCoeff struct specifies all the coefficients required to perform the interpolation. */
  class Trajectory
  {
    public:
  
    /*! \class
      \brief The TPoint class specifies a point in a trajectory. A point in a trajectory is essentially a n dimensional vector of positions and velocities with a timestamp. 
    */
    struct TPoint
    {

        TPoint() {} /** Default constructor */

        TPoint(int dimension){setDimension(dimension);}; /** Constructor with dimension specified */

        TPoint(const std::vector<double>& q, double time); /** Constructor with input of vector of positions for the point and timestamp */
            
        std::vector<double> q_; /** vector of positions of dimension = dimension_*/

        std::vector<double> qdot_; /** vector of velocities of dimension = dimension_ */

        double time_; /** timestamp */

        int dimension_; /** dimension of the point */

        friend class Trajectory;

        /*! 
          \brief Set the dimension of a trajectory point. This resizes the internal vectors to the right sizes
          \param the dimension of the trajectory point
        */
        void setDimension(int dimension){
          dimension_ = dimension;
          q_.resize(dimension_);
          qdot_.resize(dimension_);
        }
    };

    /*! \class
      \brief The TCoeff class specifies the polynomial coefficients required for interpolation between two points in a trajectory. 
    */
    struct TCoeff
    {
        TCoeff() {}

        /* 
           \brief Get the coefficient corresponding to a degree in the polynomial and a dimension index */ 
        inline double get_coefficient(int degree, int dim_index); 

        private: 

        int degree_; /** degree of the polynomial*/

        int dimension_; /** dimension of the coefficient structure */

        double duration_; /** duration of this trajectory segment */

        std::vector<std::vector<double> > coeff_; /** std::vector of coefficients */

        friend class Trajectory;
    };
  
    /*! 
       \brief Constructor for instantiation of the class by specifying the dimension 
     */
    Trajectory(int dimension);

    /*!
       \brief Destructor 
     */
    virtual ~Trajectory() {}

    /*!
      \brief clear the trajectory
    */
    void clear();

    /*!
      \brief Add a point to the trajectory
    */
    void addPoint(const TPoint);

    /*!
      \brief Set the trajectory using a vector of trajectory points
      \param std::vector of trajectory points
    */
    int setTrajectory(const std::vector<TPoint>& tp);

    /*!
      \brief Set the trajectory using a vector of values and timestamps
      \param std::vector of size dimension x number of points - specifies a list of waypoints to initialize the trajectory with
      \param std::vector of time stamps 
      \param number of points in the trajectory
    */
    int setTrajectory(const std::vector<double> &p, const std::vector<double> &time, int numPoints);

    /*!
      \brief Set the trajectory using a vector of values, timestamps are not specified and so autocalc_timing_ is set to true within this function. 
      Max rates and max accelerations should be set before this function is called.
      \param std::vector of size dimension x number of points - specifies a list of waypoints to initialize the trajectory with
      \param number of points in the trajectory
    */
    int setTrajectory(const std::vector<double> &p, int numPoints);

    int setTrajectory(const std::vector<double> &p, const std::vector<double> &pdot, const std::vector<double> &time, int numPoints);

    int setMaxAcc(std::vector<double> max_acc);


    /*!
      \brief Get the total time for the trajectory.
      \return the total time for the trajectory.
    */
    //inline double getTotalTime();
    double getTotalTime();

    /*!
      \brief Sample the trajectory at a certain point in time.
      \param reference to a pre-allocated struct of type TPoint
      \param time at which trajectory is to be sampled
      \return -1 if error, 1 if successful
    */
    int sample(TPoint &tp, double time);

//  void sample(std::vector<TPoint> &tp, double dT);

//    void sample(std::vector<TPoint> &tp, double start_time, double end_time, double dT); 

//  std::vector<TPoint>& getPoints() const;

    /*!
      \brief Set the interpolation method
      \param std::string interpolation method
    */
    void setInterpolationMethod(std::string interp_method);
 
    /*!
      \brief set the max rates (velocities) 
      \param std::vector of size dimension_ containing the max rates for the degrees of freedom in the trajectory
     */
    int setMaxRates(std::vector<double> max_rate);

    bool autocalc_timing_;/** if true, the max rates are used to compute trajectory timings, if false trajectory timings must be input by the user.*/

    /*!
      \brief Get the number of points in the trajectory
      \return number of points
    */
    int getNumberPoints();

    /*!
      \brief Minimize segment times in the trajectory
       Timings for the trajectory segments are automatically calculated using max rate and/or max accn information 
       based on the current value of interp_method_; 
    */
    int minimizeSegmentTimes();

    int getDuration(std::vector<double> &duration);

    int getDuration(int index, double &duration);

    int getTimeStamps(std::vector<double> &timestamps);

    int write(std::string filename, double dT);

    void setJointWraps(int index);

    /*! 
       \brief finds the trajectory segment corresponding to a particular time 
       \param input time (in seconds)
       \return segment index 
    */
    int findTrajectorySegment(double time);

    void getTrajectory(std::vector<trajectory::Trajectory::TPoint> &traj, double dT);

    protected:

    std::string interp_method_; /** string representation of interpolation method */

    private:

    bool max_acc_set_;

    bool max_rate_set_;

    const TPoint& lastPoint();

    void init(int num_points, int dimension);

    int num_points_; /** number of points in the trajectory */
 
    int dimension_; /** dimension of the trajectory */

    std::vector<TPoint> tp_; /** vector of TPoints in the trajectory */

    std::vector<TCoeff> tc_; /** vector of polynomial coefficients for use to define the trajectory segments*/

    std::vector<double> max_limit_;/** vector of max limits on the n DOFs of the trajectory */

    std::vector<double> min_limit_;/** vector of min limits on the n DOFs of the trajectory */

    std::vector<double> max_rate_;/** vector of max rates on the n DOFs of the trajectory */

    std::vector<double> max_acc_;/** vector of max accelerations on the n DOFs of the trajectory */

    std::vector<bool> joint_wraps_; /** boolean specifying if the joint wraps */

    /*!
      \brief calculate the coefficients for interpolation between trajectory points
       If autocalc_timing_ is true, timings for the trajectories are automatically calculated using max rate and/or max accn information
       Thus, the time duration for any trajectory segment is the maximum of two times: 
       the time duration specified by the user and the time duration dictated by the constraints. 
    */
    int parameterize();  

    /*!
      \brief calculate the coefficients for interpolation between trajectory points using linear interpolation
       If autocalc_timing_ is true, timings for the trajectories are automatically calculated using max rate information. Thus,
       the time duration for any segment is the maximum of two times: the time duration specified by the user
       and the time duration dictated by the constraints. 
    */
    int parameterizeLinear();

    /*!
      \brief calculate the coefficients for interpolation between trajectory points using blended linear interpolation.
       If autocalc_timing_ is true, timings for the trajectories are automatically calculated using max rate  and max acceleration information. Thus,
       the time duration for any segment is the maximum of two times: the time duration specified by the user
       and the time duration dictated by the constraints. 
    */
    int parameterizeBlendedLinear();

    /*!
      \brief calculate the coefficients for interpolation between trajectory points using cubic interpolation.
       If autocalc_timing_ is true, timings for the trajectories are automatically calculated using max rate information. Thus,
       the time duration for any segment is the maximum of two times: the time duration specified by the user
       and the time duration dictated by the constraints. 
    */
    int parameterizeCubic();

    /*!
      \brief calculate a minimum time trajectory using linear interpolation
       Timings for the trajectory are automatically calculated using max rate information. 
    */
    int  minimizeSegmentTimesWithLinearInterpolation();

    /*!
      \brief calculate a minimum time trajectory using cubic interpolation
       Timings for the trajectory are automatically calculated using max rate information. 
    */
    int  minimizeSegmentTimesWithCubicInterpolation();

    /*!
      \brief calculate a minimum time trajectory using blended linear interpolation
       Timings for the trajectory are automatically calculated using max rate information. 
    */
    int  minimizeSegmentTimesWithBlendedLinearInterpolation();

    /*!
       \brief Sample the trajectory based on a linear interpolation
       \param reference to pre-allocated output trajectory point
       \param time at which trajectory is being sample
       \param polynomial coefficients for this segment of the trajectory
       \param segment start time
    */
    void sampleLinear(TPoint &tp, double time, const TCoeff &tc, double segment_start_time);

    /*!
       \brief Sample the trajectory based on a cubic interpolation
       \param reference to pre-allocated output trajectory point
       \param time at which trajectory is being sample
       \param polynomial coefficients for this segment of the trajectory
       \param segment start time
    */
    void sampleCubic(TPoint &tp, double time, const TCoeff &tc, double segment_start_time);

    /*!
       \brief Sample the trajectory based on a cubic interpolation
       \param reference to pre-allocated output trajectory point
       \param time at which trajectory is being sample
       \param polynomial coefficients for this segment of the trajectory
       \param segment start time
    */
    void sampleBlendedLinear(TPoint &tp, double time, const TCoeff &tc, double segment_start_time);

    double calculateMinTimeCubic(double q0, double q1, double v0, double v1, double vmax, int index);

    /*!
      \brief calculate minimum time for a trajectory segment using a linear interpolation
      \param start TPoint
      \param end TPoint
    */
    double calculateMinimumTimeLinear(const TPoint &start, const TPoint &end);

    /*!
      \brief calculate minimum time for a trajectory segment using LSPB. 
      \param start TPoint
      \param end TPoint
    */
    double calculateMinimumTimeLSPB(const TPoint &start, const TPoint &end);

    double calculateMinTimeLSPB(double q0, double q1, double vmax, double amax, int index);

    double calculateMinimumTimeCubic(const TPoint &start, const TPoint &end);

    double blendTime(double aa,double bb,double cc);

    double jointDiff(double from, double to, int index);

  };
}

#endif
