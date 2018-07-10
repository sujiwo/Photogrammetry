/*
 * utilities.h
 *
 *  Created on: Jul 7, 2018
 *      Author: sujiwo
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_


#include <set>
#include <map>
#include <vector>

using std::set;
using std::map;
using std::vector;


template<typename P, typename Q>
map<Q,P> reverseMap (const map<P,Q> &smap)
{
	map<Q,P> nmap;
	for (auto &pq: smap) {
		P p = pq.first;
		Q q = pq.second;
		nmap[q] = p;
	}
	return nmap;
}


template<typename K, typename V>
set<K> allKeys (const map<K,V> &smap)
{
	set<K> retval;
	for (auto &p: smap) {
		retval.insert(p.first);
	}
	return retval;
}


template<typename T>
set<T> toSet (const vector<T> &vect)
{
	set<T> Res;
	for (T &v: vect) {
		Res.insert(v);
	}
	return Res;
}


template<typename T>
bool inSet (const set<T> &S, const T &val)
{
	return (S.find(val) != S.end());
}


// Result = A - B
template<typename T>
set<T> subtract (const set<T> &A, const set<T> &B)
{
	set<T> Res;
	for (auto &p: A) {
		if (!inSet(B, p)) {
			Res.insert(p);
		}
	}
	return Res;
}


template<typename K, typename V>
vector<V> allValues (const map<K,V> &sMap)
{
	vector<V> allV;
	for (auto &kv: sMap) {
		allV.push_back(kv.second);
	}
	return allV;
}


#endif /* UTILITIES_H_ */
