import pychrono as chrono
import math

# 3D vectors

v1_s = chrono.ChVectorF(1, 2, 3)  # create vector with given (x, y, z) single precision components
v1 = chrono.ChVectorD(1, 2, 3)    # create vector with given (x, y, z) double precision components
v2 = chrono.ChVectorD(4, 1, 2)    # 
v3 = chrono.ChVectorD             # create default vector (0, 0, 0)
v4 = chrono.ChVectorD(v1 + v2)    # create vector by copying from another one (result of '+' here)

v3 = v1 + v2                      # vector operations: +, -
v3 += v1                          # in-place operations
v3 = v2 * 0.03                    # multiplication by scalar (scalar to the right only)
v3 = v1 * v2                      # component-wise vector product (not common!)
v3 = v1 % v2                      # cross product operator
v3.Normalize()                    # vector functions
a1 = v1 ^ v2                      # inner product (dot product)
a3 = v1.Dot(v2)                   #   another way to do dot product
a2 = chrono.ChVectorD.Dot(v1, v2) #   yet another way to do dot product


print(' v1 = ', v1)
print(' v2 = ', v2)
print(' v3 = ', v3)
print(' v4 = ', v4)
print(' dot = ', a1, '  ', a2, '  ', a3)

# Quaternions

q1_s = chrono.ChQuaternionD(1,2,3,4)    # create quaternion with (w, x, y, z) single precision components
q1 = chrono.ChQuaternionD(1,2,3,4)      # create quaternion with (w, x, y, z) double precision components
angle = math.pi / 3
axis = chrono.ChVectorD(1, 0, 0)
q2 = chrono.Q_from_AngAxis(angle, axis) # create (unit) quaternion from angle and axis (Euler)
q2 = chrono.Q_from_AngX(angle)          # create a quaternion to represent a rotation about X axis

q1.Normalize()                          # normalize quaternion -> unit quaternion
qc2 = ~q2                               # conjugate quaternion
q3 = q1 * q2                            # quaternion product
q3 = q1 % q2                            #   another operator for quaternion product
a = q1 ^ q2                             # quaternion dot product

v = chrono.ChVectorD(1, 2, 3)
vr = q2.Rotate(v)                       # rotate a vector on the basis of this quaternion
vrr1 = q2.RotateBack(vr)                # rotate a vector on the basis of the conjughate of this quaternion
vrr2 = qc2.Rotate(vr)                   # check back rotation

print('q1  = ', q1)
print('q2  = ', q2)
print('qc2 = ', qc2)
print('q3  = ', q3)
print('dot  = ', a)

print('v   = ', v)
print('vr  = ', vr)
print('vrr1 = ', vrr1)
print('vrr2 = ', vrr2)


quit()




# Test vectors
my_vect1 = chrono.ChVectorD()
my_vect1.x=5
my_vect1.y=2
my_vect1.z=3
my_vect2 = chrono.ChVectorD(3,4,5)
my_vect4 = my_vect1*10 + my_vect2
my_len = my_vect4.Length()
print ('vect sum   =', my_vect1 + my_vect2)
print ('vect cross =', my_vect1 % my_vect2)
print ('vect dot   =', my_vect1 ^ my_vect2)

# Test quaternions
my_quat = chrono.ChQuaternionD(1,2,3,4)
my_qconjugate = ~my_quat
print ('quat. conjugate  =', my_qconjugate)
print ('quat. dot product=', my_qconjugate ^ my_quat)
print ('quat. product=',     my_qconjugate % my_quat)

# Test matrices and NumPy interoperability
mlist = [[1,2,3,4], [5,6,7,8], [9,10,11,12], [13,14,15,16]]
ma = chrono.ChMatrixDynamicD() 
ma.SetMatr(mlist)   # Create a Matrix from a list. Size is adjusted automatically.
npmat = np.asarray(ma.GetMatr()) # Create a 2D npy array from the list extracted from ChMatrixDynamic
w, v = LA.eig(npmat)  # get eigenvalues and eigenvectors using numpy
mb = chrono.ChMatrixDynamicD(4,4)
prod = v * npmat   # you can perform linear algebra operations with numpy and then feed results into a ChMatrixDynamicD using SetMatr 
mb.SetMatr(v.tolist())    # create a ChMatrixDynamicD from the numpy eigenvectors
mr = chrono.ChMatrix33D()
mr.SetMatr([[1,2,3], [4,5,6], [7,8,9]])
print  (mr*my_vect1);


# Test frames -
#  create a frame representing a translation and a rotation
#  of 20 degrees on X axis
my_frame = chrono.ChFrameD(my_vect2, chrono.Q_from_AngAxis(20*chrono.CH_C_DEG_TO_RAD, chrono.ChVectorD(1,0,0)))
my_vect5 = my_vect1 >> my_frame

# Print the class hierarchy of a chrono class
import inspect
inspect.getmro(chrono.ChStreamOutAsciiFile)



# Use the ChFunction classes
my_funct = chrono.ChFunction_Sine(0,0.5,3)
print ('function f(0.2)=', my_funct.Get_y(0.2) )


# Inherit from the ChFunction, from the Python side,
# (do not forget the __init__ constructor)

class MySquareFunct (chrono.ChFunction):
    def __init__(self):
         chrono.ChFunction.__init__(self)
    def Get_y(self,x):
         return x*x


my_funct2 = MySquareFunct()
print ('function f(2) =', my_funct2.Get_y(3) )
print ('function df/dx=', my_funct2.Get_y_dx(3) )








