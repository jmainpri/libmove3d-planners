#ifndef GRIDCUBE_H
#define GRIDCUBE_H

class Cube {
public:
    std::vector<Node*> mNodes;
    int mTries;
    int mFound;
    int mIndex;
    GridGraph* mG;
    bool mDisabled;
    SamplingAPI* mSampler;

    Cube(int index, GridGraph* graph, int tries = 0, int found = 0);
    Cube(const Cube& other) {}
    Cube& operator = (const Cube& other) { return(*this); }
    ~Cube();

    int sample(std::vector<Node*>& new_nodes, int n, int limit);
    void addNode(Node* N);
    double edgeMean();
    double outgoingMean();

};

template<class T, class STREAM>
        void print_vector(std::vector<T>& v, STREAM& stream)
{
    if(v.size() > 0)
    {
        stream << v[0];
        for(typename std::vector<T>::iterator it = v.begin()+1, end = v.end(); it < end; it++)
        {
            stream << " " << (*it);
        }
    }
    stream << std::endl;
}
template<class T, class STREAM>
        void read_vector(std::vector<T>& v, STREAM& stream)
{
    T tmp;
    while(stream.peek() != '\n')
    {
        stream >> tmp;
        v.push_back(tmp);
    }
    stream.ignore(1);
}

typedef bool (*Compare)(p3d_node* n1, p3d_node* n2);

extern void p3d_shoot_bounded(p3d_rob* R, std::vector<DofSpec>& params, shared_ptr<Configuration> q, bool sample_passive);

class GridGraph : public Graph
{
public:
    double mLength;
    std::vector<double> mOrig, mEnd;
    std::vector<uint> mDegrees;
    std::vector<uint> mFactor;
    std::vector<uint> mCumulFactor;
    uint mDim;
    std::vector<Cube*> mCubes;
    std::vector<bool> mFreeSpace;
    uint mFreeSpaceN;
    uint mCubesN;
    double mMaxDist;

    GridGraph(double length, std::vector<double> orig, std::vector<double> end, std::vector<uint> degrees,
              p3d_graph* g, Robot* r, LocalpathFactory* pathFactory, SamplingAPI* sampler, double maxDist = P3D_HUGE) :
    Graph(g, r, pathFactory, sampler),
    mLength(length), mOrig(orig), mEnd(end), mDegrees(degrees),
    mDim(mDegrees.size()), mFreeSpaceN(0), mMaxDist(maxDist)
    {
        for(uint i = 0; i < mDim; i++)
        {
            mFactor.push_back((uint)((mEnd[i] - mOrig[i]) / mLength) + 1);
            mCumulFactor.push_back(i == 0 ? 1 : mCumulFactor[i-1] * mFactor[i-1]);
        }

        mCubesN = mDim == 0 ? 0 : mCumulFactor.back() * mFactor.back();

        for(uint i(0); i < mCubesN; i++)
        {
            mCubes.push_back(new Cube(i, this));
            mFreeSpace.push_back(false);
        }
        this->initSampler();
    }

    GridGraph(const char* filename, p3d_graph* g, Robot* r, LocalpathFactory* pathFactory, SamplingAPI* sampler, double maxDist = P3D_HUGE) :
            Graph(g, r, pathFactory, sampler),
            mFreeSpaceN(0), mMaxDist(maxDist)
    {
        shared_ptr<std::fstream> gridFile = open_file(filename, std::ios_base::in);
        int tries;
        int found;
        *gridFile >> mDim;
        *gridFile >> mCubesN;
        *gridFile >> mLength;
        (*gridFile).ignore(1);
        read_vector(mDegrees, *gridFile);
        read_vector(mOrig, *gridFile);
        read_vector(mEnd, *gridFile);
        read_vector(mFactor, *gridFile);
        read_vector(mCumulFactor, *gridFile);
        uint freeCube;
        while(*gridFile >> freeCube) {
            *gridFile >> tries;
            *gridFile >> found;
            for(uint i = mCubes.size(); i < freeCube && i < mCubesN; i++) {
                mCubes.push_back(new Cube(i, this, 0, 0));
                mFreeSpace.push_back(false);
            }
            mCubes.push_back(new Cube(freeCube, this, tries, found));
            mFreeSpace.push_back(true);
            mFreeSpaceN++;
        }
        this->initSampler();
    }

    ~GridGraph()
    {
        for(uint i(0); i < mCubes.size(); i++)
            delete(mCubes[i]);
        this->Graph::freeResources();
    }

    void initSampler()
    {
        std::vector<DofSpec> params;
        for(uint i(0); i < mDim; i++) {
            params.push_back(DofSpec(mDegrees[i], mOrig[i], mEnd[i]));
        }
        mSampler = new GridSampler(mR, mR->copyConfig(mR->mR->ROBOT_GOTO), params, true);
    }

    void writeGrid(const char* filename)
    {
        shared_ptr<std::fstream> gridFile = open_file(filename, std::ios_base::out);
        *gridFile << mDim << endl;
        *gridFile << mCubesN << endl;
        *gridFile << mLength << endl;
        print_vector(mDegrees, *gridFile);
        print_vector(mOrig, *gridFile);
        print_vector(mEnd, *gridFile);
        print_vector(mFactor, *gridFile);
        print_vector(mCumulFactor, *gridFile);
        for(uint i = 0; i < mCubesN; i++)
        {
            if(mFreeSpace[i])
            {
                *gridFile << i << " " << mCubes[i]->mTries << " " << mCubes[i]->mFound << endl;
            }
        }
    }

    bool withinBounds(std::vector<double> point)
    {
        bool b(false);

        for(uint i(0); i < mDim && !b; i++)
        {
            b = b || (point[i] < mOrig[i] || point[i] > mEnd[i]);
        }

        return(!b);
    }

    int coords_to_n(Configuration& q)
    {
        int n(0);

        for(uint i(0); i < mDim; i++)
        {
            n += int ((q.mQ[mDegrees[i]] - mOrig[i]) / mLength) * mCumulFactor[i];
        }

        if(n < 0 || n >= (int) mCubesN)
            return(-1);
        else
            return(n);
    }

    std::vector<double> n_to_coords(int n)
    {
        std::vector<double> coords(mDim);

        for(int i = mDim - 1; i >= 0; i--)
        {
            coords[i] = (double) (n / mCumulFactor[i]) * mLength + mOrig[i];
            n = n % mCumulFactor[i];
        }
        return(coords);
    }

    void sampleGrid(int tries)
    {
        int remaining(tries);
        double completion(0.01);
        while(remaining > 0)
        {
            if(p3d_GetStopValue())
            {
                return;
            }

            remaining--;
            shared_ptr<Configuration> q = mSampler->sample();
            if(!mR->isInCollision(*q))
                this->markCubeFree(*q);
            // some completion feedback for the user
            if(completion < ((double) tries - (double) remaining) / (double) tries)
            {
                cout << "done: " << completion << endl;
                completion += 0.01;
            }
        }
    }

    int markCubeFree(Configuration& q)
    {
        std::vector<double> coords;

        for(uint i = 0; i < mDim; i++)
        {
            coords.push_back(q.mQ[mDegrees[i]]);
        }
        if(this->withinBounds(coords))
        {
            int n = this->coords_to_n(q);
            if(!mFreeSpace[n]) {
                mFreeSpace[n] = true;
                mFreeSpaceN++;
                cout << "cube " << mFreeSpaceN;
                for(uint i = 0; i < mDim; i++)
                {
                    cout << ", " << coords[i];
                }
                cout << endl;
            }
            return(n);
        }
        else
        { cout << "out of bounds";
            for(uint i = 0; i < mDim; i++)
            { cout << ", " << coords[i]; }
            cout << endl;
            return(-1);
        }
    }

    void sampleUnderThreshold(std::vector<Node*>& newNodes, double max_neighbours, int max_tries, uint max_nodes, int tries) {
        for(uint i = 0; i < mCubes.size(); i++)
        {
            if(mFreeSpace[i])
            {
                double cube_mean = mCubes[i]->outgoingMean();
                if(! terminationCube(*mCubes[i], max_neighbours, max_tries, max_nodes))
                {
                    mCubes[i]->sample(newNodes, 1, tries);
                }
                cout << "cube: " << i << ", mean: " << cube_mean << ", size: " << mCubes[i]->mNodes.size() << endl;
            }
        }
    }

    void sampleIso(std::vector<Node*>& newNodes, uint n_per_cube, int tries)
    {
        for(uint i(0); i < mCubes.size(); i++)
        {
            if(mFreeSpace[i])
            {
                mCubes[i]->sample(newNodes, n_per_cube, tries);
            }
        }
    }

    bool terminationCube(Cube& c, double max_neighbours, int max_tries, uint max_nodes)
    {
        return((! (c.outgoingMean() < max_neighbours &&
                   c.mTries < max_tries && c.mNodes.size() < max_nodes)) ||
               c.mDisabled);
    }

    bool terminationCondition(double max_neighbours, int max_tries, uint max_nodes)
    {
        for(uint i(0); i < mCubes.size(); i++)
        {
            if(mFreeSpace[i] && ! this->terminationCube(*mCubes[i], max_neighbours, max_tries, max_nodes))
            {
                return(false);
            }
        }
        return(true);
    }

    Node* insertNode(shared_ptr<Configuration> q)
    {
        int n;
        Node* node(NULL);

        if((n = this->coords_to_n(*q)) != -1)
        {
            int cube_n = this->markCubeFree(*q);

            if(cube_n >= 0)
            {
                node = this->Graph::insertNode(q);
                mCubes[cube_n]->addNode(node);

            }
        }
        return(node);
    }
};

#endif // GRIDCUBE_H
