from pathGen import PathGen
import matplotlib.pyplot as plt

if __name__ == '__main__':
    pg = PathGen(3)

    start_pos = [0, 0, 0]
    start_vel = [0, 0, 0]
    start_acc = [0, 0, 0]

    end_pos = [1, 1, 1]
    end_vel = [0, 0, 0]
    end_acc = [0, 0, 0]

    pg.generate_path(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc)


    pos = []
    vel = []
    acc = []

    for i in range(0,100):
        traj = pg.get_waypoints_with_scale(0.01 *i)
        pos.append(traj[0][0])
        vel.append(traj[1][0])
        acc.append(traj[2][0])
    
    print(sum(vel)*0.01)

    # plot the path vel and acc 
    plt.subplot(3, 1, 1)

    plt.plot(pos)
    plt.xlabel('Time')
    plt.ylabel('Position')

    plt.subplot(3, 1, 2)

    plt.plot(vel)
    plt.xlabel('Time')
    plt.ylabel('Velocity')

    plt.subplot(3, 1, 3)

    plt.plot(acc)
    plt.xlabel('Time')
    plt.ylabel('Acceleration')

    plt.show()  