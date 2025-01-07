alg = "CCG"

instance_abv = ["empty", "random", "rmfs_1", "rmfs_2", "rooms"]

instance_dir = './instances/movingai/'

instance_name = {"empty": "empty-8-8", 
                 "random": "random-64-64-20", 
                 "rmfs_1": "warehouse-10-20-10-2-1", 
                 "rmfs_2": "warehouse-20-40-10-2-2", 
                 "rooms": "brc202d"}

instance_id = {"empty": [1, 12, 13, 14, 15, 19, 23, 24, 25, 6], 
               "random": [1, 13, 14, 15, 18, 19, 22, 23, 25, 7], 
               "rmfs_1": [1, 10, 11, 14, 16, 17, 20, 21, 6, 7], 
               "rmfs_2": [14, 15, 18, 19, 22, 23, 4, 5, 8, 9], 
               "rooms": [14, 15, 18, 19, 22, 23, 4, 5, 8, 9]}

executable = "./bcp-mapf"

log_dir = "./log/"
print(f"mkdir -p {log_dir}")

for abv in instance_abv:
    if abv == "empty":
        for na in [10, 15, 20]:
            for ts in [1, 2, 3]:
                for i in instance_id[abv]:
                    print(f"{executable} -a {na} -s {ts} -t 600 {instance_dir}{instance_name[abv]}-random-{i}.scen >> {log_dir}{instance_name[abv]}-random-{i}-{alg}.log")
    else:
        for na in [10, 20, 30, 40, 50]:
            ts = 3
            for i in instance_id[abv]:
                print(f"{executable} -a {na} -s {ts} -t 600 {instance_dir}{instance_name[abv]}-random-{i}.scen >> {log_dir}{instance_name[abv]}-random-{i}-{alg}.log")
        for ts in [1, 2, 4, 5]:
            na = 30
            for i in instance_id[abv]:
                print(f"{executable} -a {na} -s {ts} -t 600 {instance_dir}{instance_name[abv]}-random-{i}.scen >> {log_dir}{instance_name[abv]}-random-{i}-{na}agents-{ts}ts-{alg}.log")
        
    
    