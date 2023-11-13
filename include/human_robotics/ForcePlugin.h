#ifndef _FORCE_PLUGIN_HH_
#define _FORCE_PLUGIN_HH_

#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ForcePlugin : public ModelPlugin
  {
    // Frequency of the sine wave
    private: double frequency = 1.0; // in Hz

    // Amplitude of the sine wave
    private: double amplitude = 1.0; // in N

    // Phase of the sine wave
    private: double phase = 0.0; // in radian

    public: ForcePlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for convenience.
      this->model = _model;

      // Check if the force element exists, then read the value
      if (_sdf->HasElement("force"))
        this->force = _sdf->Get<ignition::math::Vector3d>("force");

      // Read the frequency and amplitude from SDF
      if (_sdf->HasElement("frequency"))
        this->frequency = _sdf->Get<double>("frequency");

      if (_sdf->HasElement("amplitude"))
        this->amplitude = _sdf->Get<double>("amplitude");

      // Initialize the phase (optional)
      if (_sdf->HasElement("phase"))
        this->phase = _sdf->Get<double>("phase");

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ForcePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Get the simulation time and period
      double time = this->model->GetWorld()->SimTime().Double();
      double period = 1.0 / this->frequency;

      // Calculate the current value of the sine wave
      double forceMagnitude = this->amplitude * sin(2.0 * M_PI / period * time + this->phase);

      // Convert the scalar force into a vector
      ignition::math::Vector3d forceVector = forceMagnitude * this->force.Normalize();

      // Apply the sinusoidal force to the model.
      this->model->GetLink("top_wrist")->AddForce(forceVector);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // Base force to apply
    private: ignition::math::Vector3d force;
  };

  // Register this plugin with the simulator
  // GZ_REGISTER_MODEL_PLUGIN(ForcePlugin)
}
#endif
