a = [1,4,3,8,5,16,2]

test = {}

for i in range(len(a)):
    temp = {i : a[i]}
    # temp.keys = i
    # temp.values = a[i]
    test.update(temp)

res = sorted(test.items(), key=(lambda x:x[1]))
# for i in range(len(a)):
#     test[i] = a[i]

get_index = []
for p in range(4):
    get_index.append(res[p][0])

print(get_index)
