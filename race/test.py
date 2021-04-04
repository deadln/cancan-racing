def str_to_coords_lists(s):
    s = s.split('\n')
    for i in range(len(s)):
        s[i] = s[i].split()
        for j in range(len(s[i])):
            s[i][j] = float(s[i][j])
    return s[:-1]


s = ""
with open('test_ws/w1.txt', 'r') as f:
    s = f.read()
s = str_to_coords_lists(s)
print(s)
