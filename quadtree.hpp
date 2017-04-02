#ifndef _QUADTREE_HPP_
#define _QUADTREE_HPP_

#include <vector>
#include <boost/optional.hpp>

class q_point {
	public:
		float x() { return x; };
		void x(float n_x) { x = n_x; };

		float y() { return y; };
		void y(float n_y) { y = n_y; };

		void xy(float n_x, float n_y) { x = n_x; y = n_y; };
		
		bool operator==(const q_point &rhs, const q_point &lhs) { return (lhs.x == rhs.x && lhs.y == rhs.y); };
	
		q_point() {
			x = 0.0f;
			y = 0.0f;
		};

		q_point(float n_x, float n_y) {
			x = n_x;
			y = n_y;
		};

	private:
		float x;
		float y;
};

class q_box {
	public:
		// intersection operator
		bool collides(q_box b);

		// point in box operator
		bool operator[](const q_point &p) { 
			if((p.x() - center.x() < h_d) && (p.y() - center.y() < h_d)) return true;
			return false;
		};

		int quadrant(q_point p) {
			if(p.x() > center.x()) {
				if(p.y() > center.y()) {
					return 1;
				} else {
					return 4;
				}
			} else if(p.x() < center.x()) {
				if(p.y > center.y()) {
					return 2;
				} else {
					return 3;
				}
			}
			
			return 0;
		};

		const q_point & center() { return center; };
		float half_dim() { return h_d; };

		void set_point(float x, y) { center.xy(x, y); };
		void set_dim(float h_dim) { h_d = h_dim; };
		
		q_box() : center() {
			half_dim = 0.0f;
		};
		
		q_box(float center_x, float center_y, float h_dim) : center(center_x, center_y) {
			half_dim = h_dim;
		};
	private:
		q_point center;
		float h_d;
};


class q_tree {
	public:
		q_tree(float x, float y, float h_dim);

		bool add(q_point p); 
		void clean(); // removes one point from each leaf node (?)
		boost::optional<q_point> search(q_point p); // returns the closest node to p
		boost::optional< std::vector<q_point> > find_neighbors(q_point p); // returns all neighbor nodes to the node p is in
		boost::optional< std::vector<q_point> > ray_trace(q_point a, q_point b); // returns all nodes passed through by A-->B (?)
	
	private:
		struct node {
			node(float x, float y, float h_dim);
			node();
			~node();

			split();
			merge();
		
			q_box region;
			bool leaf;
			const unsigned int max_capacity;
			unsigned int capacity;

			node *parent;
			node *children[4];
		};

		q_box bounds;
		node *root, *sentinel;
		std::vector<node *> leaves;
		const int max_capacity;
		const float max_resolution;
};

#endif
