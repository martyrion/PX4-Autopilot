import sys

count = 200

def output(s):
    sys.stdout.write(s)

def var_name(i):
    """ get a variable name in the form abc from a counter i """
    b = ord('a')
    return chr(b+((i/26/26) % 26)) + chr(b+((i/26) % 26)) + chr(b+(i % 26))

for i in range(count):
    output("#define APPLY{:}(t".format(i))
    output(''.join([", {:}".format(var_name(j)) for j in range(i)]))
    output(") ")
    output(''.join(["t({:}) ".format(var_name(j)) for j in range(i)]))
    output("\n")

output('''\n#define NUM_ARGS_H1(dummy{:}, ...) x0
'''.format(''.join([', x'+str(x) for x in reversed(range(count))])))
output('''#define NUM_ARGS(...) NUM_ARGS_H1(dummy, ##__VA_ARGS__{:})
'''.format(''.join([', '+str(x) for x in reversed(range(count))])))
output('''
#define APPLY_ALL_H3(t, n, ...) APPLY##n(t, __VA_ARGS__)
#define APPLY_ALL_H2(t, n, ...) APPLY_ALL_H3(t, n, __VA_ARGS__)
#define APPLY_ALL(t, ...) APPLY_ALL_H2(t, NUM_ARGS(__VA_ARGS__), __VA_ARGS__)
''')
