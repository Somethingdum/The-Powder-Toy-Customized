#pragma once
#include <thread>
#include <vector>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <algorithm>

// A small persistent pool for splitting a row range [begin, end) into
// contiguous chunks processed in parallel. The calling thread does one chunk
// itself (worker 0) and helper threads do the rest, so a pool of N threads
// spawns N-1 helpers. ForRows blocks until every chunk is done.
//
// Intended for stencil passes whose cells are independent within the pass
// (reads come from arrays not written by that pass, or from a separate output
// buffer), so row-block partitioning yields bit-identical results to a serial
// loop. It does NOT make order-dependent (in-place neighbour-read/write)
// passes safe.
class RowWorkerPool
{
public:
	explicit RowWorkerPool(int threads) : threadCount(std::max(1, threads))
	{
		for (int i = 1; i < threadCount; i++)
		{
			workers.emplace_back([this, i] { WorkerLoop(i); });
		}
	}

	~RowWorkerPool()
	{
		{
			std::lock_guard<std::mutex> lk(mtx);
			stop = true;
			generation++;
		}
		cvWork.notify_all();
		for (auto &t : workers)
		{
			t.join();
		}
	}

	RowWorkerPool(const RowWorkerPool &) = delete;
	RowWorkerPool &operator=(const RowWorkerPool &) = delete;

	int Threads() const
	{
		return threadCount;
	}

	// Splits [begin, end) into Threads() contiguous chunks (remainder spread
	// across the first chunks) and runs body(chunkBegin, chunkEnd) on each.
	// Blocks until all chunks finish.
	void ForRows(int begin, int end, const std::function<void(int, int)> &fn)
	{
		if (threadCount == 1 || end - begin <= 1)
		{
			fn(begin, end);
			return;
		}
		{
			std::lock_guard<std::mutex> lk(mtx);
			curBegin = begin;
			curEnd = end;
			body = &fn;
			remaining = threadCount;
			generation++;
		}
		cvWork.notify_all();
		RunChunk(0); // calling thread is worker 0
		std::unique_lock<std::mutex> lk(mtx);
		cvDone.wait(lk, [this] { return remaining == 0; });
		body = nullptr;
	}

private:
	void RunChunk(int idx)
	{
		int total = curEnd - curBegin;
		int chunk = total / threadCount;
		int rem = total % threadCount;
		int cb = curBegin + idx * chunk + std::min(idx, rem);
		int ce = cb + chunk + (idx < rem ? 1 : 0);
		if (cb < ce)
		{
			(*body)(cb, ce);
		}
		std::lock_guard<std::mutex> lk(mtx);
		if (--remaining == 0)
		{
			cvDone.notify_one();
		}
	}

	void WorkerLoop(int idx)
	{
		int lastGen = 0;
		for (;;)
		{
			std::unique_lock<std::mutex> lk(mtx);
			cvWork.wait(lk, [this, &lastGen] { return generation != lastGen; });
			lastGen = generation;
			if (stop)
			{
				return;
			}
			lk.unlock();
			RunChunk(idx);
		}
	}

	const int threadCount;
	std::vector<std::thread> workers;
	std::mutex mtx;
	std::condition_variable cvWork;
	std::condition_variable cvDone;
	const std::function<void(int, int)> *body = nullptr;
	int curBegin = 0;
	int curEnd = 0;
	int remaining = 0;
	int generation = 0;
	bool stop = false;
};
