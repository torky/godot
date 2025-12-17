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

#include <cmath>
#include <type_traits>

#include "Jolt/Core/STLLocalAllocator.h"
#include "Jolt/Physics/Collision/InternalEdgeRemovingCollector.h"
#include "Jolt/Physics/Collision/Shape/Shape.h"
#include "Jolt/Physics/PhysicsSettings.h"

// Type traits to detect body ID members for deterministic tie-breaking
template <typename T, typename = void>
struct has_mBodyID : std::false_type {};

template <typename T>
struct has_mBodyID<T, std::void_t<decltype(std::declval<T>().mBodyID)>> : std::true_type {};

template <typename T, typename = void>
struct has_mBodyID2 : std::false_type {};

template <typename T>
struct has_mBodyID2<T, std::void_t<decltype(std::declval<T>().mBodyID2)>> : std::true_type {};

// Helper to extract a sortable body ID from any hit type
template <typename Hit>
inline uint64_t GetHitBodySortKey(const Hit &hit) {
	if constexpr (has_mBodyID<Hit>::value) {
		return hit.mBodyID.GetIndexAndSequenceNumber();
	} else if constexpr (has_mBodyID2<Hit>::value) {
		return hit.mBodyID2.GetIndexAndSequenceNumber();
	} else {
		return 0; // No tie-breaking available
	}
}

// Check if two fractions are close enough to be considered equal for determinism
// Uses Jolt's collision tolerance (1e-4f) as the threshold
inline bool AreFractionsEqual(float a, float b) {
	const float diff = std::abs(a - b);
	return diff <= JPH::cDefaultCollisionTolerance;
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
	bool valid = false;
	// Debug: track tie-breaking info
	int tie_count = 0;
	int total_hits_considered = 0;

public:
	bool had_hit() const { return valid; }

	const Hit &get_hit() const { return hit; }

	int get_tie_count() const { return tie_count; }
	int get_total_hits_considered() const { return total_hits_considered; }

	void reset() {
		Reset();
	}

	virtual void Reset() override {
		TBase::Reset();
		valid = false;
		tie_count = 0;
		total_hits_considered = 0;
	}

	virtual void AddHit(const Hit &p_hit) override {
		total_hits_considered++;
		const float early_out = p_hit.GetEarlyOutFraction();
		const float current_fraction = hit.GetEarlyOutFraction();

		if (!valid) {
			// First hit - accept it
			TBase::UpdateEarlyOutFraction(early_out);
			hit = p_hit;
			valid = true;
		} else if (AreFractionsEqual(early_out, current_fraction)) {
			// Tie (within epsilon) - use BodyID as secondary sort key for determinism (lower BodyID wins)
			tie_count++;
			if (GetHitBodySortKey(p_hit) < GetHitBodySortKey(hit)) {
				hit = p_hit;
				// Update early_out to use the smaller fraction for consistency
				if (early_out < current_fraction) {
					TBase::UpdateEarlyOutFraction(early_out);
				}
			}
		} else if (early_out < current_fraction) {
			// Strictly closer - accept it
			TBase::UpdateEarlyOutFraction(early_out);
			hit = p_hit;
		}
		// else: further away, ignore
	}
};

template <typename TBase, int TDefaultCapacity>
class JoltQueryCollectorClosestMulti final : public TBase {
public:
	typedef typename TBase::ResultType Hit;
	typedef JPH::Array<Hit, JPH::STLLocalAllocator<Hit, TDefaultCapacity + 1>> HitArray;

private:
	HitArray hits;
	int max_hits = 0;

public:
	explicit JoltQueryCollectorClosestMulti(int p_max_hits = TDefaultCapacity) :
			max_hits(p_max_hits) {
		hits.reserve(TDefaultCapacity + 1);
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
		const float new_fraction = p_hit.GetEarlyOutFraction();
		typename HitArray::const_iterator E = hits.cbegin();
		for (; E != hits.cend(); ++E) {
			const float existing_fraction = E->GetEarlyOutFraction();
			if (AreFractionsEqual(new_fraction, existing_fraction)) {
				// Tie (within epsilon) - use BodyID as secondary sort key for determinism (lower BodyID first)
				if (GetHitBodySortKey(p_hit) < GetHitBodySortKey(*E)) {
					break;
				}
			} else if (new_fraction < existing_fraction) {
				// Strictly closer - insert before
				break;
			}
		}

		hits.insert(E, p_hit);

		if ((int)hits.size() > max_hits) {
			hits.resize(max_hits);
		}
	}
};
