from tf import transformations as tfm

roll, pitch, yaw = 0.046039, 0.002832, 4.923170
q = tfm.quaternion_from_euler(roll, pitch, yaw)
# q = [-0.01878777, 0.0133726, 0.6286571, -0.77734063]
print(q)