import os, sys

for filename in sys.argv[1:]:
    with open(filename, 'rb') as f:
        data = f.read()
    name = os.path.basename(filename).replace('.', '_').upper()
    print(f'static const char {name}[] = {{')
    chunks = [data[i:i+16] for i in range(0, len(data), 16)]
    for chunk in chunks:
        print('  ' + ', '.join(f'0x{b:02x}' for b in chunk) + ',')
    print('};')
