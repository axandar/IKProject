namespace Sources{
	using UnityEngine;

	public class RobotJoint : MonoBehaviour{
		public Vector3 Axis;
		public Vector3 StartOffset;

		private void Awake (){
			StartOffset = transform.localPosition;
		}
	}
 }