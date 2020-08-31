/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <stdlib.h>

template<class T>
class IntrusiveQueue
{
public:

	bool empty() const { return _head == nullptr; }

	T front() const { return _head; }
	T back() const { return _tail; }

	size_t size() const
	{
		size_t sz = 0;

		for (auto node = front(); node != nullptr; node = node->next_intrusive_queue_node()) {
			sz++;
		}

		return sz;
	}

	void push(T newNode)
	{
		// error, node already queued or already inserted
		if ((newNode->next_intrusive_queue_node() != nullptr) || (newNode == _tail)) {
			return;
		}

		if (_head == nullptr) {
			_head = newNode;
		}

		if (_tail != nullptr) {
			_tail->set_next_intrusive_queue_node(newNode);
		}

		_tail = newNode;
	}

	T pop()
	{
		T ret = _head;

		if (!empty()) {
			if (_head != _tail) {
				_head = _head->next_intrusive_queue_node();

			} else {
				// only one item left
				_head = nullptr;
				_tail = nullptr;
			}

			// clear next in popped (in might be re-inserted later)
			ret->set_next_intrusive_queue_node(nullptr);
		}

		return ret;
	}

	bool remove(T removeNode)
	{
		// base case
		if (removeNode == _head) {
			if (_head->next_intrusive_queue_node() != nullptr) {
				_head = _head->next_intrusive_queue_node();
				removeNode->set_next_intrusive_queue_node(nullptr);

			} else {
				_head = nullptr;
				_tail = nullptr;
			}

			return true;
		}

		for (T node = _head; node != nullptr; node = node->next_intrusive_queue_node()) {
			// is sibling the node to remove?
			if (node->next_intrusive_queue_node() == removeNode) {
				if (removeNode == _tail) {
					_tail = node;
				}

				// replace sibling
				node->set_next_intrusive_queue_node(removeNode->next_intrusive_queue_node());
				removeNode->set_next_intrusive_queue_node(nullptr);
				return true;
			}
		}

		return false;
	}

	struct Iterator {
		T node;
		Iterator(T v) : node(v) {}

		operator T() const { return node; }
		operator T &() { return node; }
		T operator* () const { return node; }
		Iterator &operator++ ()
		{
			if (node) {
				node = node->next_intrusive_queue_node();
			};

			return *this;
		}
	};

	Iterator begin() { return Iterator(_head); }
	Iterator end() { return Iterator(nullptr); }

private:

	T _head{nullptr};
	T _tail{nullptr};

};

template<class T>
class IntrusiveQueueNode
{
private:
	friend IntrusiveQueue<T>;

	T next_intrusive_queue_node() const { return _next_intrusive_queue_node; }
	void set_next_intrusive_queue_node(T new_next) { _next_intrusive_queue_node = new_next; }

	T _next_intrusive_queue_node{nullptr};
};
