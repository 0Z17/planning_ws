from pathGen import PathGen

if __name__ == '__main__':
    pg = PathGen(3)

    start_pos = [0, 0, 0]
    start_vel = [0, 0, 0]
    start_acc = [0, 0, 0]

    end_pos = [1, 1, 1]
    end_vel = [0, 0, 0]
    end_acc = [0, 0, 0]

    pg.generate_path(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc)

    print(pg.duration)

    pg.generate_path(end_pos, end_vel, end_acc, start_pos, start_vel, start_acc)

    print(pg.duration)    

    print(pg.get_waypoints_with_scale(0.2))