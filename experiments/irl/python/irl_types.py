import gtsam
from collections import namedtuple
from gtsam.symbol_shorthand import X, L
import numpy as np

"""
##     ## ########    ###     ######  ##     ## ########  ######## ##     ## ######## ##    ## ########  ######  
###   ### ##         ## ##   ##    ## ##     ## ##     ## ##       ###   ### ##       ###   ##    ##    ##    ## 
#### #### ##        ##   ##  ##       ##     ## ##     ## ##       #### #### ##       ####  ##    ##    ##       
## ### ## ######   ##     ##  ######  ##     ## ########  ######   ## ### ## ######   ## ## ##    ##     ######  
##     ## ##       #########       ## ##     ## ##   ##   ##       ##     ## ##       ##  ####    ##          ## 
##     ## ##       ##     ## ##    ## ##     ## ##    ##  ##       ##     ## ##       ##   ###    ##    ##    ## 
##     ## ######## ##     ##  ######   #######  ##     ## ######## ##     ## ######## ##    ##    ##     ######  
"""


def serialize_pose(dim, lin, pose):
    if lin == "nonlinear":
        if dim == 3:
            rx, ry, rz = pose.rotation().xyz()
            return "{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}".format(
                pose.x(), pose.y(), pose.z(), rx, ry, rz
            )
        else:
            return "{:.5f} {:.5f} {:.5f}".format(pose.x(), pose.y(), pose.theta())
    else:
        if dim == 3:
            return "{:.5f} {:.5f} {:.5f}".format(pose[0], pose[1], pose[2])
        else:
            return "{:.5f} {:.5f}".format(pose[0], pose[1])


def serialize_measurements(measurements):
    return " ".join(map(lambda m: m.serialize(), measurements))


def parse_measurements(dim, lin, num_hypos, tokens, MeasureClass):
    measurements = []
    elems = MeasureClass.num_tokens(dim, lin)
    for i in range(num_hypos):
        if tokens[i * elems] == "NULL":
            measurements.append(NullHypo())
        else:
            measurements.append(
                MeasureClass.parse(dim, lin, tokens[i * elems : (i + 1) * elems])
            )
    return measurements


class NullHypo:
    def serialize(self):
        return "NULL"


class PoseMeasure:
    def __init__(self, dim, lin, pose, covariance):
        self.dim = dim
        self.linearity = lin
        self.pose = pose
        self.covariance = covariance

    def serialize(self):
        return "{} {}".format(
            serialize_pose(self.dim, self.linearity, self.pose),
            " ".join(map(str, self.covariance.flatten())),
        )

    @classmethod
    def parse(cls, dim, lin, tokens):
        if lin == "nonlinear":
            if dim == 3:
                x, y, z, rx, ry, rz = map(float, tokens[:6])
                pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(rx, ry, rz), np.array([x, y, z]))
                covariance = np.array(list(map(float, tokens[6:]))).reshape((6, 6))
            else:
                pose = gtsam.Pose2(*map(float, tokens[:3]))
                covariance = np.array(list(map(float, tokens[3:]))).reshape((3, 3))
        else:
            if dim == 3:
                x, y, z = map(float, tokens[:3])
                pose = gtsam.Point3(np.array([x, y, z]))
                covariance = np.array(list(map(float, tokens[3:]))).reshape((3, 3))
            else:
                pose = gtsam.Point2(*map(float, tokens[:2]))
                covariance = np.array(list(map(float, tokens[2:]))).reshape((2, 2))
        return cls(dim, lin, pose, covariance)

    @classmethod
    def num_tokens(cls, dim, lin):
        if lin == "nonlinear":
            if dim == 3:
                return 6 + 6 * 6
            else:
                return 3 + 3 * 3
        else:
            if dim == 3:
                return 3 + 3 * 3
            else:
                return 2 + 2 * 2


class LoopMeasure:
    def __init__(self, dim, lin, pose_b_key, pose, covariance):
        self.pose_b_key = pose_b_key
        self.dim = dim
        self.linearity = lin
        self.pose = pose
        self.covariance = covariance

    def serialize(self):
        symb = gtsam.Symbol(self.pose_b_key)
        return "{} {} {}".format(
            symb.index(),
            serialize_pose(self.dim, self.linearity, self.pose),
            " ".join(map(str, self.covariance.flatten())),
        )

    @classmethod
    def parse(cls, dim, lin, tokens):
        k = X(int(tokens[0]))
        if lin == "nonlinear":
            if dim == 3:
                x, y, z, rx, ry, rz = map(float, tokens[1:7])
                pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(rx, ry, rz), np.array([x, y, z]))
                covariance = np.array(list(map(float, tokens[7:]))).reshape((6, 6))
            else:
                pose = gtsam.Pose2(*map(float, tokens[1:4]))
                covariance = np.array(list(map(float, tokens[4:]))).reshape((3, 3))
        else:
            if dim == 3:
                x, y, z = map(float, tokens[1:4])
                pose = gtsam.Point3(np.array([x, y, z]))
                covariance = np.array(list(map(float, tokens[4:]))).reshape((3, 3))
            else:
                pose = gtsam.Point2(*map(float, tokens[1:3]))
                covariance = np.array(list(map(float, tokens[3:]))).reshape((2, 2))
        return cls(dim, lin, k, pose, covariance)

    @classmethod
    def num_tokens(cls, dim, lin):
        if lin == "nonlinear":
            if dim == 3:
                return 7 + 6 * 6
            else:
                return 4 + 3 * 3
        else:
            if dim == 3:
                return 4 + 3 * 3
            else:
                return 3 + 2 * 2


"""
######## ##    ## ######## ########  #### ########  ######  
##       ###   ##    ##    ##     ##  ##  ##       ##    ## 
##       ####  ##    ##    ##     ##  ##  ##       ##       
######   ## ## ##    ##    ########   ##  ######    ######  
##       ##  ####    ##    ##   ##    ##  ##             ## 
##       ##   ###    ##    ##    ##   ##  ##       ##    ## 
######## ##    ##    ##    ##     ## #### ########  ######  
"""


class Prior:
    def __init__(self, num_modes, correct_mode, key, measurements):
        self.num_modes = num_modes
        self.correct_mode = correct_mode
        self.key = key
        self.measurements = measurements

    def serialize(self):
        symb = gtsam.Symbol(self.key)
        return "PRIOR {} {} {} {}".format(
            self.num_modes,
            self.correct_mode,
            symb.index(),
            serialize_measurements(self.measurements),
        )

    @classmethod
    def parse(cls, dim, lin, tokens):
        return cls(
            int(tokens[0]),
            int(tokens[1]),
            X(int(tokens[2])),
            parse_measurements(dim, lin, int(tokens[0]), tokens[3:], PoseMeasure),
        )


class Odometry:
    def __init__(self, num_modes, correct_mode, start_key, end_key, measurements):
        self.num_modes = num_modes
        self.correct_mode = correct_mode
        self.start_key = start_key
        self.end_key = end_key
        self.measurements = measurements

    def serialize(self):
        s = gtsam.Symbol(self.start_key)
        e = gtsam.Symbol(self.end_key)
        return "ODOMETRY {} {} {} {} {}".format(
            self.num_modes,
            self.correct_mode,
            s.index(),
            e.index(),
            serialize_measurements(self.measurements),
        )

    @classmethod
    def parse(cls, dim, lin, tokens):
        return cls(
            int(tokens[0]),
            int(tokens[1]),
            X(int(tokens[2])),
            X(int(tokens[3])),
            parse_measurements(dim, lin, int(tokens[0]), tokens[4:], PoseMeasure),
        )


class Loop:
    def __init__(self, num_modes, correct_mode, pose_a_key, measurements):
        self.num_modes = num_modes
        self.correct_mode = correct_mode
        self.pose_a_key = pose_a_key
        self.measurements = measurements

    def serialize(self):
        s = gtsam.Symbol(self.pose_a_key)
        return "LOOP {} {} {} {}".format(
            self.num_modes,
            self.correct_mode,
            s.index(),
            serialize_measurements(self.measurements),
        )

    @classmethod
    def parse(cls, dim, lin, tokens):
        return cls(
            int(tokens[0]),
            int(tokens[1]),
            X(int(tokens[2])),
            parse_measurements(dim, lin, int(tokens[0]), tokens[3:], LoopMeasure),
        )

"""
########  #######  ########     ##       ##     ## ##       
   ##    ##     ## ##     ##    ##       ##     ## ##       
   ##    ##     ## ##     ##    ##       ##     ## ##       
   ##    ##     ## ########     ##       ##     ## ##       
   ##    ##     ## ##           ##        ##   ##  ##       
   ##    ##     ## ##           ##         ## ##   ##       
   ##     #######  ##           ########    ###    ######## 
"""


class Header:
    def __init__(self, name, date, dim, lin, usr_str):
        self.name = name
        self.date = date
        self.dim = dim
        self.linearity = lin
        self.usr_str = usr_str

    def serialize(self):
        return "{}\n{}\n{}\n{}\n{}".format(
            self.name, self.date, self.dim, self.linearity, self.usr_str
        )

    @classmethod
    def parse(cls, lines):
        return cls(
            lines[0].strip(),
            lines[1].strip(),
            int(lines[2].strip()),
            lines[3].strip(),
            lines[4].strip(),
        )


class Log:
    def __init__(self, header, entries):
        self.header = header
        self.entries = entries

    def write(self, out_file):
        with open(out_file, "w") as f:
            f.write(self.header.serialize() + "\n")
            for entry in self.entries:
                f.write(entry.serialize() + "\n")

    @classmethod
    def read(cls, in_file):
        with open(in_file) as f:
            lines = f.readlines()
            header = Header.parse(lines[0:5])

            entries = []
            for line in lines[5:]:
                tokens = line.strip().split(" ")
                if tokens[0] == "PRIOR":
                    entries.append(
                        Prior.parse(header.dim, header.linearity, tokens[1:])
                    )
                elif tokens[0] == "ODOMETRY":
                    entries.append(
                        Odometry.parse(header.dim, header.linearity, tokens[1:])
                    )
                elif tokens[0] == "LOOP":
                    entries.append(Loop.parse(header.dim, header.linearity, tokens[1:]))
                else:
                    raise Exception(
                        "IRL Log.read found unknown token: {}".format(tokens[0])
                    )
            return cls(header, entries)
