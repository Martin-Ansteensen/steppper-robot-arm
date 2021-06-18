# /usr/bin/env python3
# A python version of https://github.com/glumb/robot-gui/blob/master/js/InverseKinematic.js
import math

class Kinematics:
    def __init__(self, geometry):
        #  geometry is an [5][3] Array including geometry information [x,y,z]
        if len(geometry) != 5:
            print("geometry array must have 5 entries")
            # Throw error
        conditions = [
            geometry[3][1] != 0,
            geometry[3][2] != 0,
            geometry[4][0] != 0,
            geometry[4][2] != 0,
        ]
        if True in conditions:
            print("geometry 3 and 4 must be one dimensional geo[3] = [a,0,0] geo[4] = [0,b,0]")
            # Throw error
        self.V1_length_x_y = math.sqrt((geometry[1][0] ** 2) + (geometry[1][1] ** 2))
        self.V4_length_x_y_z = math.sqrt((geometry[4][0] ** 2) + (geometry[4][1] ** 2) + (geometry[4][2] ** 2))
        
        self.geometry = geometry

        self.J_initial_absolute = []
        tmpPos = [0, 0, 0]
        for i in range(len(geometry)):
            self.J_initial_absolute.append([tmpPos[0], tmpPos[1], tmpPos[2]])
            tmpPos[0] += geometry[i][0]
            tmpPos[1] += geometry[i][1]
            tmpPos[2] += geometry[i][2]
        
        self.R_corrected = [0, 0, 0, 0, 0, 0]

        self.R_corrected[1] -= math.pi / 2
        self.R_corrected[1] += math.atan2(geometry[1][0], geometry[1][1]) # correct offset bone

        self.R_corrected[2] -= math.pi / 2
        self.R_corrected[2] -= math.atan2((geometry[2][1] + geometry[3][1]), (geometry[2][0] + geometry[3][0])) # correct offset bone V2,V3
        self.R_corrected[2] -= math.atan2(geometry[1][0], geometry[1][1]) # correct bone offset of V1

        self.R_corrected[4] += math.atan2(geometry[4][1], geometry[4][0])

    def inverse(self, x, y, z, a, b, c): # Position and rotation
        ca = math.cos(a)
        sa = math.sin(a)
        cb = math.cos(b)
        sb = math.sin(b)
        cc = math.cos(c)
        sc = math.sin(c) 
        
        targetVectorX = [
            cb * cc,
            cb * sc, -sb,
        ]

        R = [
            self.R_corrected[0],
            self.R_corrected[1],
            self.R_corrected[2],
            self.R_corrected[3],
            self.R_corrected[4],
            self.R_corrected[5],
        ]
        J = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]
        # ---- J5 ----
        J[5][0] = x
        J[5][1] = y
        J[5][2] = z

        # ---- J4 ----
        J[4][0] = x - self.V4_length_x_y_z * targetVectorX[0]
        J[4][1] = y - self.V4_length_x_y_z * targetVectorX[1]
        J[4][2] = z - self.V4_length_x_y_z * targetVectorX[2]


        # ---- R0 ----
        # J4

        R[0] += math.pi / 2 - math.acos(self.J_initial_absolute[4][2] / Kinematics.length2(J[4][2], J[4][0]))
        R[0] += math.atan2(-J[4][2], J[4][0])

        if self.J_initial_absolute[4][2] > Kinematics.length2(J[4][2], J[4][0]):
            print("out of reach")

        # ---- J1 ----
        # R0

        J[1][0] = math.cos(R[0]) * self.geometry[0][0] + math.sin(R[0]) * self.geometry[0][2]
        J[1][1] = self.geometry[0][1]
        J[1][2] = -math.sin(R[0]) * self.geometry[0][0] + math.cos(R[0]) * self.geometry[0][2]


        # ---- rotate J4 into x,y plane ----
        # J4 R0

        J4_x_y = []

        J4_x_y[0] = math.cos(R[0]) * J[4][0] + -math.sin(R[0]) * J[4][2]
        J4_x_y[1] = J[4][1]
        J4_x_y[2] = math.sin(R[0]) * J[4][0] + math.cos(R[0]) * J[4][2]

        # ---- J1J4_projected_length_square ----
        # # J4 R0

        J1J4_projected_length_square = ((J4_x_y[0] - self.J_initial_absolute[1][0]) ** 2) + ((J4_x_y[1] - self.J_initial_absolute[1][1]) ** 2) # not using math.sqrt

        # ---- R2 ----
        # J4 R0

        J2J4_length_x_y = Kinematics.length2(self.geometry[2][0] + self.geometry[3][0], self.geometry[2][1] + self.geometry[3][1])
        R[2] += math.acos((-J1J4_projected_length_square + (J2J4_length_x_y ** 2) + (self.V1_length_x_y ** 2)) / (2.0 * (J2J4_length_x_y) * self.V1_length_x_y))

        # ---- R1 ----
        # # J4 R0

        J1J4_projected_length = math.sqrt(J1J4_projected_length_square)
        R[1] += math.atan2((J4_x_y[1] - self.J_initial_absolute[1][1]), (J4_x_y[0] - self.J_initial_absolute[1][0]))
        R[1] += math.acos((+J1J4_projected_length_square - (J2J4_length_x_y ** 2) + (self.V1_length_x_y ** 2)) / (2.0 * J1J4_projected_length * self.V1_length_x_y))

        # ---- J2 ----
        # R1 R0

        ta = math.cos(R[0])
        tb = math.sin(R[0])
        tc = self.geometry[0][0]
        d = self.geometry[0][1]
        e = self.geometry[0][2]
        f = math.cos(R[1])
        g = math.sin(R[1])
        h = self.geometry[1][0]
        i = self.geometry[1][1]
        j = self.geometry[1][2]
        k = math.cos(R[2])
        l = math.sin(R[2])
        m = self.geometry[2][0]
        n = self.geometry[2][1]
        o = self.geometry[2][2]

        J[2][0] = ta * tc + tb * e + ta * f * h - ta * g * i + tb * j
        J[2][1] = d + g * h + f * i
        J[2][2] = -tb * tc + ta * e - tb * f * h + tb * g * i + ta * j

        # ---- J3 ----
        # # R0 R1 R2

        J[3][0] = ta * tc + tb * e + ta * f * h - ta * g * i + tb * j + ta * f * k * m - ta * g * l * m - ta * g * k * n - ta * f * l * n + tb * o
        J[3][1] = d + g * h + f * i + g * k * m + f * l * m + f * k * n - g * l * n
        J[3][2] = -tb * tc + ta * e - tb * f * h + tb * g * i + ta * j - tb * f * k * m + tb * g * l * m + tb * g * k * n + tb * f * l * n + ta * o

        # ---- J4J3 J4J5 ----
        # J3 J4 J5

        J4J5_vector = [J[5][0] - J[4][0], J[5][1] - J[4][1], J[5][2] - J[4][2]]
        J4J3_vector = [J[3][0] - J[4][0], J[3][1] - J[4][1], J[3][2] - J[4][2]]

        # ---- R3 ----
        # J3 J4 J5

        J4J5_J4J3_normal_vector = Kinematics.cross(J4J5_vector, J4J3_vector)
        XZ_parallel_aligned_vector = [
            10 * math.cos(R[0] + (math.pi / 2)),
            0, -10 * math.sin(R[0] + (math.pi / 2)),
        ]

        reference = Kinematics.cross(XZ_parallel_aligned_vector, J4J3_vector)

        R[3] = Kinematics.angleBetween(J4J5_J4J3_normal_vector, XZ_parallel_aligned_vector, reference)

        # ---- R4 ----
        # J4 J3 J5

        referenceVector = Kinematics.cross(J4J3_vector, J4J5_J4J3_normal_vector)

        R[4] += Kinematics.angleBetween(J4J5_vector, J4J3_vector, referenceVector)

        # ---- R5 ----
        # J3 J4 J5

        targetVectorY = [
            sa * sb * cc - sc * ca,
            sa * sb * sc + cc * ca,
            sa * cb,
        ]

        R[5] += math.pi / 2
        R[5] -= Kinematics.angleBetween(J4J5_J4J3_normal_vector, targetVectorY, Kinematics.cross(targetVectorY, targetVectorX))

        # --- return angles ---
        return R

    def calculateTCP(self, R0, R1, R2, R3, R4, R5, jointsResult):
        joints = self.calculateCoordinates(R0, R1, R2, R3, R4, R5)
        jointsResult[0] = joints[5][0]
        jointsResult[1] = joints[5][1]
        jointsResult[2] = joints[5][2]
        jointsResult[3] = joints[5][3]
        jointsResult[4] = joints[5][4]
        jointsResult[5] = joints[5][5]
    

    def forward(self, R0, R1, R2, R3, R4, R5): # Angle of each joint
        a = math.cos(R0)
        b = math.sin(R0)
        c = self.geometry[0][0]
        d = self.geometry[0][1]
        e = self.geometry[0][2]
        f = math.cos(R1)
        g = math.sin(R1)
        h = self.geometry[1][0]
        i = self.geometry[1][1]
        j = self.geometry[1][2]
        k = math.cos(R2)
        l = math.sin(R2)
        m = self.geometry[2][0]
        n = self.geometry[2][1]
        o = self.geometry[2][2]
        p = math.cos(R3)
        q = math.sin(R3)
        r = self.geometry[3][0]
        s = self.geometry[3][1]
        t = self.geometry[3][2]
        u = math.cos(R4)
        v = math.sin(R4)
        w = self.geometry[4][0]
        x = self.geometry[4][1]
        y = self.geometry[4][2]
        A = math.cos(R5)
        B = math.sin(R5)
        #  C = 0 // self.geometry[5][0]
        #  D = 0 // self.geometry[5][1]
        #  E = 0 // self.geometry[5][2]

        jointsResult = [[0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0], [0,0,0,0,0,0]]

        jointsResult[0][0] = 0
        jointsResult[0][1] = 0
        jointsResult[0][2] = 0

        jointsResult[1][0] = jointsResult[0][0] + a * c + b * e
        jointsResult[1][1] = jointsResult[0][1] + d
        jointsResult[1][2] = jointsResult[0][2] + -b * c + a * e

        jointsResult[2][0] = jointsResult[1][0] + a * f * h - a * g * i + b * j
        jointsResult[2][1] = jointsResult[1][1] + g * h + f * i
        jointsResult[2][2] = jointsResult[1][2] - b * f * h + b * g * i + a * j

        jointsResult[3][0] = jointsResult[2][0] + a * f * k * m - a * g * l * m - a * g * k * n - a * f * l * n + b * o
        jointsResult[3][1] = jointsResult[2][1] + g * k * m + f * l * m + f * k * n - g * l * n
        jointsResult[3][2] = jointsResult[2][2] - b * f * k * m + b * g * l * m + b * g * k * n + b * f * l * n + a * o

        jointsResult[4][0] = jointsResult[3][0] + a * f * k * r - a * g * l * r - a * g * k * p * s - a * f * l * p * s + b * q * s + b * p * t + a * g * k * q * t + a * f * l * q * t
        jointsResult[4][1] = jointsResult[3][1] + g * k * r + f * l * r + f * k * p * s - g * l * p * s - f * k * q * t + g * l * q * t
        jointsResult[4][2] = jointsResult[3][2] - b * f * k * r + b * g * l * r + b * g * k * p * s + b * f * l * p * s + a * q * s + a * p * t - b * g * k * q * t - b * f * l * q * t

        jointsResult[5][0] = jointsResult[4][0] + a * f * k * u * w - a * g * l * u * w - a * g * k * p * v * w - a * f * l * p * v * w + b * q * v * w - a * g * k * p * u * x - a * f * l * p * u * x + b * q * u * x - a * f * k * v * x + a * g * l * v * x + b * p * y + a * g * k * q * y + a * f * l * q * y
        jointsResult[5][1] = jointsResult[4][1] + g * k * u * w + f * l * u * w + f * k * p * v * w - g * l * p * v * w + f * k * p * u * x - g * l * p * u * x - g * k * v * x - f * l * v * x - f * k * q * y + g * l * q * y
        jointsResult[5][2] = jointsResult[4][2] - b * f * k * u * w + b * g * l * u * w + b * g * k * p * v * w + b * f * l * p * v * w + a * q * v * w + b * g * k * p * u * x + b * f * l * p * u * x + a * q * u * x + b * f * k * v * x - b * g * l * v * x + a * p * y - b * g * k * q * y - b * f * l * q * y

        M = [
        [a * g * k * p * u + a * f * l * p * u - b * q * u + a * f * k * v - a * g * l * v,
            -B * b * p - B * a * g * k * q - B * a * f * l * q + A * a * f * k * u - A * a * g * l * u - A * a * g * k * p * v - A * a * f * l * p * v + A * b * q * v,
            A * b * p + A * a * g * k * q + A * a * f * l * q + B * a * f * k * u - B * a * g * l * u - B * a * g * k * p * v - B * a * f * l * p * v + B * b * q * v],
        [-f * k * p * u + g * l * p * u + g * k * v + f * l * v,
            B * f * k * q - B * g * l * q + A * g * k * u + A * f * l * u + A * f * k * p * v - A * g * l * p * v,
            -A * f * k * q + A * g * l * q + B * g * k * u + B * f * l * u + B * f * k * p * v - B * g * l * p * v],
        [-b * g * k * p * u - b * f * l * p * u - a * q * u - b * f * k * v + b * g * l * v,
            -B * a * p + B * b * g * k * q + B * b * f * l * q - A * b * f * k * u + A * b * g * l * u + A * b * g * k * p * v + A * b * f * l * p * v + A * a * q * v,
            A * a * p - A * b * g * k * q - A * b * f * l * q - B * b * f * k * u + B * b * g * l * u + B * b * g * k * p * v + B * b * f * l * p * v + B * a * q * v],
        ]


        yaw = 0
        pitch = 0
        roll = 0
        if M[2][0] != 1 or M[2][0] != -1:
            yaw = math.pi + math.asin(M[2][0])
            pitch = math.atan2(M[2][1] / math.cos(yaw), M[2][2] / math.cos(yaw))
            roll = math.atan2(M[1][0] / math.cos(yaw), M[0][0] / math.cos(yaw))
        else:
            roll = 0 # anything; can set to
            if M[2][0] == -1: 
                yaw = math.pi / 2
                pitch = roll + math.atan2(M[0][1], M[0][2])
            else:
                yaw = -math.pi / 2
                pitch = -roll + math.atan2(-M[0][1], -M[0][2])


        jointsResult[5][3] = pitch
        jointsResult[5][4] = yaw
        jointsResult[5][5] = roll

        return jointsResult

    def cross(self, vectorA, vectorB):
        return [
        vectorA[1] * vectorB[2] - vectorA[2] * vectorB[1],
        vectorA[2] * vectorB[0] - vectorA[0] * vectorB[2],
        vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0],
        ]

    def dot(self, vectorA, vectorB):
        return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1] + vectorA[2] * vectorB[2]

    def angleBetween(self, vectorA, vectorB, referenceVector):
        # angle = atan2(norm(cross(a, b)), dot(a, b))

        norm = Kinematics.length3(Kinematics.cross(vectorA, vectorB))

        angle = math.atan2(norm, (self.dot(vectorA, vectorB)))

        tmp = referenceVector[0] * vectorA[0] + referenceVector[1] * vectorA[1] + referenceVector[2] * vectorA[2]

        if tmp > 0:
            sign = 1.0
        else:
            sign = -1.0

        return angle * sign


    def length3(self, vector):
        return math.sqrt((vector[0] ** 2) + (vector[1] ** 2) + (vector[2] ** 2))

    def length2(self, a, b):
        return math.sqrt((a ** 2) + (b ** 2))

geo = [
    [1,1,10], # V0
    [0,0,10], # V1
    [1,0,3],  # V2
    [7,0,0],  # V3
    [0,0,0]   # V4
]

kin = Kinematics(geo)
position = kin.forward(0,0,0,0,0,0)
print(position)