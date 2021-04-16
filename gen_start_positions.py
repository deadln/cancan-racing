n = int(input("Enter the number of drones "))
with open('start_positions.txt', 'w') as f:
    x = 8.5
    y = 8.0
    step_x = 2
    step_y = 2
    for i in range(n):
        f.write(f'{x} {y}\n')
        y -= step_y
        if y < -8:
            x -= step_x
            y = 8.0
