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
#include <utility>
#include <Eigen/Core>
#include <Eigen/Geometry>


using std::pair;
using std::set;
using std::map;
using std::vector;
using Eigen::Quaterniond;
using Eigen::Vector3d;



struct DataItem {
	std::string imagePath;
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};


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


template<typename Scalar>
using VectorXx = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;


template <typename Scalar>
double median(const VectorXx<Scalar> &v)
{
	int n = v.rows();
	vector<Scalar> vs (v.data(), v.data()+v.rows());
	sort(vs.begin(), vs.end());
	if (n%2==1)
		return (double)vs[(n-1)/2];
	else
		return (double(vs[n/2])+double(vs[(n/2)-1])) / 2;
}


inline
int medianx (const Eigen::VectorXi &v)
{
	return median(v);
}


template <typename k, typename v>
pair<const k,v>
maximumMapElement(const map<k,v> &maptg)
{
	auto p = max_element(maptg.begin(), maptg.end(),
		[](const pair<k,v> &element1, const pair<k,v> &element2)
			{return element1.second < element2.second;}
	);
	return *p;
}


/*
 * All angles are in Radian
 */
inline Quaterniond fromRPY (double roll, double pitch, double yaw)
{
	int
		i = 0,
		j = 1,
		k = 2;

	roll /= 2.0;
	pitch /= 2.0;
	yaw /= 2.0;
	double
		ci = cos(roll),
		si = sin(roll),
		cj = cos(pitch),
		sj = sin(pitch),
		ck = cos(yaw),
		sk = sin(yaw),
		cc = ci*ck,
		cs = ci*sk,
		sc = si*ck,
		ss = si*sk;
	Quaterniond q;
	q.x() = cj*sc - sj*cs;
	q.y() = cj*ss + sj*cc;
	q.z() = cj*cs - sj*sc;
	q.w() = cj*cc + sj*ss;
	return q;
}


inline Vector3d quaternionToRPY (const Quaterniond &q)
{

}

#endif /* UTILITIES_H_ */
