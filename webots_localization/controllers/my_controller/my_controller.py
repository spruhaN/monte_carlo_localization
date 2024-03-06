"""CPE416 Sample Controller"""

from controller import Robot, Motor
from collections import Counter
import random
import math
import time

def print_histogram(values):
    # Create a dictionary to count the frequency of each value
    values = [int(num) for num in values]

    bucket_size = 20
    min_value, max_value = 0, 360
    
    # Initialize the buckets
    buckets = [0] * ((max_value - min_value) // bucket_size)
    
    # Assign each value to a bucket
    for value in values:
        if min_value <= value < max_value:
            bucket_index = (value - min_value) // bucket_size
            buckets[bucket_index] += 1
    
    # Print the histogram
    for i, count in enumerate(buckets):
        bucket_min = i * bucket_size
        bucket_max = bucket_min + bucket_size
        print(f"[{bucket_min}-{bucket_max}): {'*' * count}")
    # print("=========")


def compute_proportional(sensor_left,sensor_right):
    error = 0 - (sensor_right - sensor_left)

    output = (.9 * error)

    left_motor = 20 - output
    right_motor = 20 + output

    left_motor = constrain(left_motor, 0, 100)
    right_motor = constrain(right_motor,0, 100)

    return left_motor,right_motor


def constrain(value,minVal,maxVal):
    if value < minVal:
        return minVal
    elif value > maxVal:
        return maxVal
    else:
        return value


def in_range(actual, expected):
    lower_bound = (expected - OFFSET) % 360
    upper_bound = (expected + OFFSET) % 360

    if lower_bound < upper_bound:
        return lower_bound < actual < upper_bound
    else:
        return actual < upper_bound or actual > lower_bound

def what_space(location):
    for tower in towers:
        if in_range(location, tower):
            return True
    return False


def trap_prob(a,b,c,d,location):
    pts = [a,b,c,d]
    u = 2/(d  + c - b - a )
    if b <= location and location < c:
        return u
    if a <= location and location < b:
        return u * ((location-a)/(b-a))
    if c <= location and location < d:
        return u * ((d-location)/(d-c))
    return 0

def new_sleep(sleep_time):
    current_time_1 = float(robot.getTime())
    current_time_2 = float(robot.getTime())

    while current_time_2 < (current_time_1 + sleep_time):
        current_time_2 = float(robot.getTime())
        robot.step(1)

def turn_right():
    #turn 
    left_motor.setVelocity(1)
    right_motor.setVelocity(0)
    new_sleep(5)
    # go straight 
    while True:
        left_motor.setVelocity(1)
        right_motor.setVelocity(1)
        new_sleep(5)

def calculate_particles(new_particles):
    sin_val = 0
    cos_val = 0

    for angle in new_particles:
        sin_val += math.sin(math.radians(angle))
        cos_val += math.cos(math.radians(angle))

    sin_val /= len(new_particles)
    cos_val /= len(new_particles)

    inner = -math.log((sin_val ** 2)+(cos_val ** 2))
    if inner < 0:
        return False
    stddev = math.sqrt(inner)

    mean = math.degrees(math.atan2(sin_val, cos_val))

    if(cos_val > 0 and sin_val < 0): 
        mean += 360
    elif(cos_val < 0):
        mean += 180

    return stddev, mean


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# enable the drive motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# enable ground color sensors
left_ground_sensor = robot.getDevice('gs0')
left_ground_sensor.enable(timestep)

middle_ground_sensor = robot.getDevice('gs1')
middle_ground_sensor.enable(timestep)

right_ground_sensor = robot.getDevice('gs2')
right_ground_sensor.enable(timestep)

right_distance_sensor = robot.getDevice('ps2') #IR sensor pointing to the right
right_distance_sensor.enable(timestep)

# initialize encoders
encoders = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoders.append(robot.getDevice(encoderNames[i]))
    encoders[i].enable(timestep)


# Main loop:
# - perform simulation steps until Webots stops the controller
    

# free space range 55 - 56.04 - 78.57 - 80 a < d 
# block 90 - 102.87 - 119.13 - 121
    
OFFSET = 11.35
THRESHOLD = 1
CIRCLE_ENCODER = 59.434 # the right encoder 

towers = [0,90,270]
# 348.65 - 11.35
# 78.65 - 101.35
# 168.65 - 191.35

target_tower = 1 # 90
location = False

# generate 100 particles
particles = [15.850517836761862, 76.06815421368766, 349.59537364800116, 223.8879781071353, 243.42276923364741, 135.74427834889022, 228.39573302717963, 117.95318149357077, 271.4616631454017, 168.2655838551114, 282.38998783269716, 85.24711727240697, 211.84837482467026, 40.539628744582146, 205.37538950006544, 289.3350470906324, 284.2847730768327, 316.5412008616593, 83.23410244790487, 135.40932928280395, 30.831843663375338, 298.38479720552647, 95.06322443936845, 173.8444530833104, 274.1161132766744, 306.5800018970962, 3.8710713359182547, 262.9704533462341, 145.85227407162617, 332.4540615481167, 133.8371811842114, 297.7348752846196, 265.95170835624674, 69.10854835283885, 233.64722784628057, 68.39871422817438, 178.70920988834067, 238.6308081192492, 267.3722375570258, 164.53552725192614, 359.1796442800648, 118.03322464733968, 218.15361070924308, 10.292525350622498, 325.7957175007257, 123.01791769822243, 168.16148180732958, 329.95477412436827, 340.36053673415546, 232.53912315347216, 221.6477035207698, 300.95384374352386, 51.90083364887412, 212.6777851528236, 246.18686706134505, 286.2983522632843, 326.86230772836285, 233.66562195322692, 209.9451339729647, 352.77151318249776, 242.01256299909218, 158.34512597224116, 16.053982491782385, 53.3415812070855, 46.07766363010513, 336.3010164362628, 119.4710344042281, 122.66119360778812, 154.8355612147286, 300.502179912511, 53.941465397962396, 211.37269347888835, 194.37937994749288, 83.88856590446359, 267.30658196888544, 318.61554355596553, 306.5359008177688, 95.9023704527893, 354.81925878642113, 294.1905091286191, 251.83702327479256, 347.7207384772042, 10.055268187057566, 208.5499436579072, 99.5259732067016, 356.66883810854404, 171.29902134400626, 2.8553389338773534, 71.62924663537187, 192.95500949200735, 106.6690987545704, 303.97752990455047, 64.06122213050621, 21.92139300626656, 280.23081682187546, 188.7695960147641, 192.55598573526802, 342.2746265191758, 61.731181899017876, 325.66944392179636]
# random.sample(range(360), 100)
n = 0
encoder_values = None
# print("MCL: " + str(encoder_values))
dist = []

# run it every 5 times/32 ms and while location is not determined
while robot.step(timestep) != -1 and location is False: # every 32 ms
    # ========================= PID CONTROLLER ===============================

    # get ir sensors 
    left_sensor = left_ground_sensor.getValue()
    right_sensor = right_ground_sensor.getValue()
    # pid
    left_speed,right_speed = compute_proportional(left_sensor,right_sensor);
    # scale motor and call
    left_speed = left_speed/100
    right_speed = right_speed/100
    left_motor.setVelocity(left_speed * 5) # set the left motor (radians/second)
    right_motor.setVelocity(right_speed * 5) # set the right motor (radians/second) 

    # ========================================================================
    
    if n == 19:
        print_histogram(particles)
    # ================================= MCL ==================================
        # get distance sensor reading
        distance = right_distance_sensor.getValue()
        # print("distance: " + str(distance))



        # checks each particle if it is in free space or block and appends probability
        particle_weights = []
        for particle in particles:
            # check if particle is in free space or a block
            space = what_space(particle)

            # appends probability of distance based on category
            if space == True: # block => p(particle|block)
                weight = trap_prob(85,127,148,200 , distance) # 96, 137, 225, 250,
                particle_weights.append(weight)
                #print(str(particle) + " is a block w/ a weight of " + str(weight))
            else: # free space => p(particle|free_space)
                weight = trap_prob(40,54,78,100, distance) # 25, 56, 71, 130,
                particle_weights.append(weight)
                #print(str(particle) + " is a free_space w/ a weight of " + str(weight))

        sum_weight = sum(particle_weights)
        normalized_weights = [weight / sum_weight for weight in particle_weights]

        # choose 95 particles based on the weights
        new_particles = random.choices(particles, weights=normalized_weights, k=95)

        #print(normalized_weights)
        stdev,mean = calculate_particles(new_particles)
        if stdev is False:
            stdev = float("inf")
        if stdev < .1: # CHANGE THRESHOLD
            location =  mean # mean
            print("ANGLE IS " + str(mean))
            right_motor.setVelocity(0)
            left_motor.setVelocity(0)
            encoder_values = [encoder.getValue() for encoder in encoders]

        # adds 5 random particles for a total of a 100 new particles 
        for i in range(5):
            new_particles.append(random.uniform(0, 360))



        # getting avg diff in encoders
        curr_encoder_values = [encoder.getValue() for encoder in encoders]
        if encoder_values is None:
            diff = .32 # ?
        else:
            diff = ((curr_encoder_values[0] - encoder_values[0]) + (curr_encoder_values[1] - encoder_values[1])) / 2
        encoder_values = curr_encoder_values
        diff = (diff/CIRCLE_ENCODER) * 360
        # print("diff: " + str(diff))
        #print("MCL:" + str([encoder.getValue() for encoder in encoders]))

        # advancing particles
        for i,particle in enumerate(new_particles): 
            # motion noise
            u_1 = random.random()
            u_2 = random.random()
            z = math.sqrt(-2 * math.log(u_1)) * math.cos(2 * math.pi * u_2)

            # advancing particles w/ noise and encoder offset
            particles[i]= (particle + z + diff) % 360 # adds motion noise and avg distance between ten iter
        print(sum(particle_weights))
        print("==========" + "stdev: " + str(stdev) + "==============")
    # ========================================================================

    n += 1
    n = n % 20 # resets counter if 5 => 0 

print("DONE" + str(location))




# ========================== GO TO TOWER AND HIT  =================================

# if fin degree in range of target tower then there!
curr_degree = encoder_values[1]/59.434 * 360
while robot.step(timestep) != -1  and in_range(curr_degree, towers[target_tower]) is False:
    # ========================= PID CONTROLLER ===============================

    # get ir sensors 
    left_sensor = left_ground_sensor.getValue()
    right_sensor = right_ground_sensor.getValue()
    # pid
    left_speed,right_speed = compute_proportional(left_sensor,right_sensor);
    # scale motor and call
    left_speed = left_speed/100
    right_speed = right_speed/100
    left_motor.setVelocity(left_speed) # set the left motor (radians/second)
    right_motor.setVelocity(right_speed) # set the right motor (radians/second) 

    # ========================================================================

    # ========================= ENCODER DIFF ===============================
    
    encoder_values = [encoder.getValue() for encoder in encoders]
    curr_degree = encoder_values[1]/59.434 * 360

    # ========================================================================

turn_right()







    # print(left_ground_sensor.getValue())
    # print(middle_ground_sensor.getValue())
    # print(right_ground_sensor.getValue())
    # print(right_distance_sensor.getValue())
    # new_encoder_values = [encoder.getValue() for encoder in encoders]
    # print(new_encoder_values)
    # print('-------------------------')
    # call robot.getTime() to get the current simulation time in seconds
