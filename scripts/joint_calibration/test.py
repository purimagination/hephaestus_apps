import pinocchio
 
urdf_filename = "/home/hzx/Projects/hephaestus_ws/src/hephaestus_description/urdf/hephaestus_description.urdf"
model = pinocchio.buildModelFromUrdf(urdf_filename)
print("model name: " + model.name)
 
# Create data required by the algorithms
data = model.createData()
 
# Sample a random configuration
q = pinocchio.randomConfiguration(model)
print("q: %s" % q.T)
 
# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)
 
# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)))