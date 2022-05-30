import time
import multiprocessing
import concurrent.futures
start = time.perf_counter()

def do_something(seconds):
    print(f'Sleeping {seconds} second(s) ...')
    time.sleep(seconds)
    return f'Done sleeping ... {seconds}'

"""
# pass function not return of that function so no ()
p1 = multiprocessing.Process(target=do_something)
p2 = multiprocessing.Process(target=do_something)

p1.start()
p2.start()

# join means finish process before moving down the script
p1.join()
p2.join()

finish  = time.perf_counter()

print(f'Finished in {round(finish-start,2)} seconds')


processes = []

for _ in range (10):
    p = multiprocessing.Process(target=do_something, args=[1.5])
    p.start()
    processes.append(p)

for process in processes:
    process.join() # wait until done before continue rest of script
finish  = time.perf_counter()
print(f'Finished in {round(finish - start, 2)} seconds')


with concurrent.futures.ProcessPoolExecutor() as executer: # with won't exit until everything below is finished
    secs = [5,4,3,2,1]
    #results = [executer.submit(do_something, sec) for sec in secs]
    #for f in concurrent.futures.as_completed(results): # prints out result as they are completed
    #    print(f.result())
    results = executer.map(do_something, secs) # prints out results as they are ordered in
    for result in results:
        print(result)


finish  = time.perf_counter()
print(f'Finished in {round(finish-start,2)} seconds')
"""
