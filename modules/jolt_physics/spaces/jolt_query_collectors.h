/**************************************************************************/
/*  jolt_query_collectors.h                                               */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "../jolt_project_settings.h"
#include "jolt_space_3d.h"

#include "Jolt/Jolt.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

#include "Jolt/Core/STLLocalAllocator.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Physics/Collision/InternalEdgeRemovingCollector.h"
#include "Jolt/Physics/Collision/Shape/Shape.h"

inline float jolt_query_tie_inclusive_fraction(float p_fraction, float p_current_early_out) {
	return std::min(std::nextafter(p_fraction, std::numeric_limits<float>::infinity()), p_current_early_out);
}

template <typename TBase, int TDefaultCapacity>
class JoltQueryCollectorAll final : public TBase {
public:
	typedef typename TBase::ResultType Hit;
	typedef JPH::Array<Hit, JPH::STLLocalAllocator<Hit, TDefaultCapacity>> HitArray;

private:
	HitArray hits;

public:
	JoltQueryCollectorAll() {
		hits.reserve(TDefaultCapacity);
	}

	bool had_hit() const {
		return !hits.is_empty();
	}

	int get_hit_count() const {
		return hits.size();
	}

	const Hit &get_hit(int p_index) const {
		return hits[p_index];
	}

	void reset() { Reset(); }

	virtual void Reset() override {
		TBase::Reset();
		hits.clear();
	}

	virtual void AddHit(const Hit &p_hit) override {
		hits.push_back(p_hit);
	}
};

template <typename TBase>
class JoltQueryCollectorAny final : public TBase {
public:
	typedef typename TBase::ResultType Hit;

private:
	Hit hit;
	bool valid = false;

public:
	bool had_hit() const { return valid; }

	const Hit &get_hit() const { return hit; }

	void reset() {
		Reset();
	}

	virtual void Reset() override {
		TBase::Reset();
		valid = false;
	}

	virtual void AddHit(const Hit &p_hit) override {
		hit = p_hit;
		valid = true;

		TBase::ForceEarlyOut();
	}
};

template <typename TBase, int TDefaultCapacity>
class JoltQueryCollectorAnyMulti final : public TBase {
public:
	typedef typename TBase::ResultType Hit;
	typedef JPH::Array<Hit, JPH::STLLocalAllocator<Hit, TDefaultCapacity>> HitArray;

private:
	HitArray hits;
	int max_hits = 0;

public:
	explicit JoltQueryCollectorAnyMulti(int p_max_hits = TDefaultCapacity) :
			max_hits(p_max_hits) {
		hits.reserve(TDefaultCapacity);
	}

	bool had_hit() const {
		return hits.size() > 0;
	}

	int get_hit_count() const {
		return hits.size();
	}

	const Hit &get_hit(int p_index) const {
		return hits[p_index];
	}

	void reset() {
		Reset();
	}

	virtual void Reset() override {
		TBase::Reset();
		hits.clear();
	}

	virtual void AddHit(const Hit &p_hit) override {
		if ((int)hits.size() < max_hits) {
			hits.push_back(p_hit);
		}

		if ((int)hits.size() == max_hits) {
			TBase::ForceEarlyOut();
		}
	}
};

template <typename TBase>
class JoltQueryCollectorClosest final : public TBase {
public:
	typedef typename TBase::ResultType Hit;

private:
	Hit hit;
	uint64_t hit_body_order = 0;
	uint64_t current_body_order = 0;
	bool valid = false;
	int total_hits_considered = 0;

public:
	bool had_hit() const { return valid; }

	const Hit &get_hit() const { return hit; }

	int get_total_hits_considered() const { return total_hits_considered; }

	void reset() {
		Reset();
	}

	virtual void Reset() override {
		TBase::Reset();
		valid = false;
		total_hits_considered = 0;
		hit_body_order = 0;
		current_body_order = 0;
	}

	virtual void OnBody(const JPH::Body &p_body) override {
		TBase::OnBody(p_body);
		current_body_order = p_body.GetCreationOrder();
	}

	virtual void AddHit(const Hit &p_hit) override {
		total_hits_considered++;
		const float early_out = p_hit.GetEarlyOutFraction();
		const float current_fraction = hit.GetEarlyOutFraction();

		const bool closer = !valid || early_out < current_fraction;
		const bool tie_won = valid && early_out == current_fraction && current_body_order < hit_body_order;
		if (!closer && !tie_won) {
			return;
		}

		hit = p_hit;
		hit_body_order = current_body_order;
		valid = true;
		TBase::UpdateEarlyOutFraction(jolt_query_tie_inclusive_fraction(early_out, TBase::GetEarlyOutFraction()));
	}
};

template <typename TBase, int TDefaultCapacity>
class JoltQueryCollectorClosestMulti final : public TBase {
public:
	typedef typename TBase::ResultType Hit;
	typedef JPH::Array<Hit, JPH::STLLocalAllocator<Hit, TDefaultCapacity + 1>> HitArray;

private:
	HitArray hits;
	JPH::Array<uint64_t, JPH::STLLocalAllocator<uint64_t, TDefaultCapacity + 1>> hit_body_orders;
	uint64_t current_body_order = 0;
	int max_hits = 0;

public:
	explicit JoltQueryCollectorClosestMulti(int p_max_hits = TDefaultCapacity) :
			max_hits(p_max_hits) {
		hits.reserve(TDefaultCapacity + 1);
		hit_body_orders.reserve(TDefaultCapacity + 1);
	}

	bool had_hit() const {
		return hits.size() > 0;
	}

	int get_hit_count() const {
		return hits.size();
	}

	const Hit &get_hit(int p_index) const {
		return hits[p_index];
	}

	void reset() {
		Reset();
	}

	virtual void Reset() override {
		TBase::Reset();
		hits.clear();
		hit_body_orders.clear();
		current_body_order = 0;
	}

	virtual void OnBody(const JPH::Body &p_body) override {
		TBase::OnBody(p_body);
		current_body_order = p_body.GetCreationOrder();
	}

	virtual void AddHit(const Hit &p_hit) override {
		const float new_fraction = p_hit.GetEarlyOutFraction();
		int index = 0;
		for (; index < (int)hits.size(); ++index) {
			const float existing_fraction = hits[index].GetEarlyOutFraction();
			if (new_fraction == existing_fraction) {
				if (current_body_order < hit_body_orders[index]) {
					break;
				}
			} else if (new_fraction < existing_fraction) {
				break;
			}
		}

		hits.insert(hits.begin() + index, p_hit);
		hit_body_orders.insert(hit_body_orders.begin() + index, current_body_order);

		if ((int)hits.size() > max_hits) {
			hits.resize(max_hits);
			hit_body_orders.resize(max_hits);
		}
	}
};
