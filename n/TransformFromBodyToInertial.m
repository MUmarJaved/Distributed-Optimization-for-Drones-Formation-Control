function vector_inertial = TransformFromBodyToInertial(vector_body, euler_angles)

vector_inertial = RotationMatrix321(euler_angles)'*vector_body;