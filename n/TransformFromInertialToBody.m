function vector_body = TransformFromInertialToBody(vector_inertial, euler_angles)

vector_body = RotationMatrix321(euler_angles)*vector_inertial;