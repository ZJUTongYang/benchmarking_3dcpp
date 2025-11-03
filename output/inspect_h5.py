import h5py

def inspect_h5(filename):
    with h5py.File(filename, 'r') as f:

        print("Keys (root level): ", list(f.keys()))

        # Print file-level attributes
        print("\n[FILE LEVEL ATTRIBUTES]")
        if f.attrs:
            for key, value in f.attrs.items():
                print(f"  {key}: {value}")
        else:
            print("  (No attributes at file level)")

        def print_struture(name, obj):
            print(name)

            if isinstance(obj, h5py.Group):
                for key in obj.keys():
                    print(f"  Group: {key}")

            elif isinstance(obj, h5py.Dataset):
                print(f"  Dataset: {obj.name}, shape: {obj.shape}, dtype: {obj.dtype}")


            # Print attributes for both groups and datasets
            if obj.attrs:
                print("  Attributes:")
                for key, value in obj.attrs.items():
                    # 处理可能包含二进制字符串的情况
                    if isinstance(value, bytes):
                        try:
                            value_str = value.decode('utf-8')
                            print(f"    {key}: {value_str}")
                        except UnicodeDecodeError:
                            print(f"    {key}: <binary data, length={len(value)}>")
                    else:
                        print(f"    {key}: {value}")
            else:
                print("  (No attributes)")

        f.visititems(print_struture)

inspect_h5("circular_tool_remeshed_saddle_Yang2023Template.h5")
