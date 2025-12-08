import h5py
import numpy as np

np.set_printoptions(threshold=np.inf) # visualize all elements

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

the_shared_folder = "/home/yt/benchmark_ws/install/benchmarking_3dcpp/share/benchmarking_3dcpp/output/"
filename= "circular_remeshed_saddle_Yang2023Template_cpu.h5"
inspect_h5(the_shared_folder + filename)

# Check the covered situation per surface point. For debugging
with h5py.File(the_shared_folder+filename, 'r') as f:
    point_covered_num_dataset = f["coverage_result"]["point_covered_num"]
    point_covered_num_data = point_covered_num_dataset[:]
    print(f"point_covered_num: {point_covered_num_data}")

    coverage_indices_dataset = f["coverage_indices"]["coverage_indices"]
    coverage_indices_data = coverage_indices_dataset[:]
    print(f"coverage_indices: {coverage_indices_data}")




