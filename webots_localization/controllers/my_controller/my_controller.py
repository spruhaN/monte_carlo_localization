"""CPE416 Sample Controller"""

from controller import Robot, Motor
import random
import math

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
    

# free space range 55 - 56.04 - 78.57 - 80
# block 90 - 102.87 - 119.13 - 121
    
OFFSET = 12.5
THRESHOLD = 1


# OFFSET 12.5
towers = [0,90,225]
target_tower = 2 - 1
location = False

# generate 100 particles
particles = random.sample(range(360), 100)

while robot.step(timestep) != -1 and in_range(location, towers[target_tower]): # every 32 ms

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


    if location is False:
    # ================================= MCL ==================================

        # get distance sensor reading
        distance = right_distance_sensor.getValue()

        # checks each particle if it is in free space or block and appends probability
        particle_weights = []
        for particle in particles:
            space = what_space(particle)
            if space == 0 : # free space => p(particle|free_space)
                particle_weights.append(trap_prob(90, 102.87, 119.13, 121, particle))
            else: # block => p(particle|block)
                particle_weights.append(trap_prob(55, 56.04, 78.57, 80, particle))


        # normalizes sum to .95 
        sum_weight = sum(particle_weights)
        normalized_weights = [weight / sum_weight * 0.95 for weight in particle_weights]

        # appends weights to spinner w/ 5% of random
        spinner = []
        for i,weight in enumerate(normalized_weights):
            for i in range(5):
                spinner.append("RANDOM")
            for i in range(weight * 100):
                spinner.append(particles[i])
        
        # spins and resamples for new particles
        for i in range(100):
            spin = random.sample(range(100)) % 100
            if spinner[spin] == "RANDOM": # if random randomize particle 0-360
                particles[i] = random.sample(range(360))
            else:
                particles[i] = spinner[spin]

        # if stdev is less than threshold than we found location else False
        mean = sum(particles) / len(particles) 
        variance = sum([((x - mean) ** 2) for x in particles]) / len(particles) 
        stdev = variance ** 0.5
        location = False
        if stdev < 5: # CHANGE THRESHOLD
            location = mean
    # ========================================================================


    # ============================ MOTION NOISE ==============================

    u_1 = random.sample(range(100))/100
    u_2 = random.sample(range(100))/100
    z = math.isqrt(-2 * math.log(u_1)) * math.cos(2 * math.pi() * u_2)
    # advancing both location if found and particles if not
    if location is not False:
        location += z
    else:
        particles = [(particle + z)%360 for particle in particles]

    # ========================================================================



# ============================== HIT TOWER ====================================
    
if location == towers[target_tower]:
    # found tower turn and hit
    # turn NEED TO FINETUNE SLEEP
    left_motor.setVelocity(0)
    right_motor.setVelocity(1)
    sleep(10)
    # straight
    left_motor.setVelocity(1)
    right_motor.setVelocity(1)

# =============================================================================








    # print(left_ground_sensor.getValue())
    # print(middle_ground_sensor.getValue())
    # print(right_ground_sensor.getValue())
    # print(right_distance_sensor.getValue())
    # new_encoder_values = [encoder.getValue() for encoder in encoders]
    # print(new_encoder_values)
    # print('-------------------------')
    # call robot.getTime() to get the current simulation time in seconds




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
    

def if_range(actual, expected):
    return (actual < (expected + OFFSET)% 360 and actual > (expected - OFFSET) % 360)

def what_space(location):
    for tower in towers:
        if if_range(location, tower) is True:
            return True
    return False

def trap_prob(a,b,c,d,location):
    u = 2/ (d  + c - b - a )
    if location < a:
        return 0
    if b <= location and location < c:
        return u
    if a <= location and location < b:
        return u * ((location-a)/(b-a))
    if a <= location and location < b:
        return u * ((d-location)/(d-c))