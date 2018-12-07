using System;
using System.Collections.Generic;
using UnityEngine;

namespace Sources{
	public class IKLogic : MonoBehaviour{

		public RobotJoint[] Joints;
		public float SamplingDistance;
		public float LearningRate;
		public Transform Target;
		public float DistanceThreshold;

		private float[] _angles;
	
		private void Start(){
			_angles = new float[Joints.Length];
			for(var i = 0; i < Joints.Length; i++){
				var joint = Joints[i];
				if(Math.Abs(joint.Axis.x - 1) < 0.001){
					_angles[i] = Joints[i].transform.localRotation.x;
				}else if(Math.Abs(joint.Axis.y - 1) < 0.001){
					_angles[i] = Joints[i].transform.localRotation.y;
				}else if(Math.Abs(joint.Axis.z - 1) < 0.001){
					_angles[i] = Joints[i].transform.localRotation.z;
				}
			}
		}

		private void FixedUpdate(){
			InverseKinematics(Target.position, _angles);
			Debug.Log("--------------------------------------");
			for(var i = 0; i < Joints.Length; i++){
				var joint = Joints[i];
				Debug.Log(_angles[i] + "  --  " + GetAxisNameFromVector(joint.Axis));
			
				//joint.transform.rotation = Quaternion.AngleAxis(_angles[i], joint.Axis);
				if(Math.Abs(joint.Axis.x - 1) < 0.001){
					joint.transform.localEulerAngles = new Vector3(_angles[i], 0, 0);
				}else if(Math.Abs(joint.Axis.y - 1) < 0.001){
					joint.transform.localEulerAngles = new Vector3(0, _angles[i], 0);
				}else if(Math.Abs(joint.Axis.z - 1) < 0.001){
					joint.transform.localEulerAngles = new Vector3(0, 0, _angles[i]);
				}
			}
		}

		private string GetAxisNameFromVector(Vector3 axis){
			if(Math.Abs(axis.x - 1) < 0.001){
				return "x";
			}
			if(Math.Abs(axis.y - 1) < 0.001){
				return "y";
			}
			if(Math.Abs(axis.z - 1) < 0.001){
				return "z";
			}

			return "";
		}

		public void InverseKinematics (Vector3 target, float[] angles){
			if(DistanceFromTarget(target, angles) < DistanceThreshold){
				return;
			}

			for (var i = 0; i < Joints.Length; i ++){
				// Gradient descent
				// Update : Solution -= LearningRate * Gradient
				var gradient = PartialGradient(target, angles, i);
				angles[i] -= LearningRate * gradient;

				if(DistanceFromTarget(target, angles) < DistanceThreshold){
					return;
				}
			}
		}

		private float PartialGradient (Vector3 target, IList<float> angles, int i){
			var angle = angles[i];
 
			// Gradient : [F(x+SamplingDistance) - F(x)] / h
			var f_x = DistanceFromTarget(target, angles);
 
			angles[i] += SamplingDistance;
			var f_x_plus_d = DistanceFromTarget(target, angles);
 
			var gradient = (f_x_plus_d - f_x) / SamplingDistance;
 
			// Restores
			angles[i] = angle;
 
			return gradient;
		}

		private float DistanceFromTarget(Vector3 target, IList<float> angles){
			var point = ForwardKinematics (angles);
			return Vector3.Distance(point, target);
		}

		private Vector3 ForwardKinematics (IList<float> angles){
			var prevPoint = Joints[0].transform.position;
			var rotation = Quaternion.identity;
			for (var i = 1; i < Joints.Length; i++){
				// Rotates around a new axis
				rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].Axis);
				var nextPoint = prevPoint + rotation * Joints[i].StartOffset;
    
				prevPoint = nextPoint;
			}
			return prevPoint;
		}
	}
}
