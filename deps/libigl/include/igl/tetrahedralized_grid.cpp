#include "tetrahedralized_grid.h"
#include "grid.h"

template <
  typename DerivedGV,
  typename DerivedGT>
IGL_INLINE void igl::tetrahedralized_grid(
  const int nx,
  const int ny,
  const int nz,
  const TetrahedralizedGripType type,
  Eigen::PlainObjectBase<DerivedGV> & GV,
  Eigen::PlainObjectBase<DerivedGT> & GT)
{
  Eigen::RowVector3i res(nx,ny,nz);
  igl::grid(res,GV);
  return igl::tetrahedralized_grid(GV,res,GT);
}

template <
  typename DerivedGV,
  typename Derivedside,
  typename DerivedGT>
IGL_INLINE void igl::tetrahedralized_grid(
  const Eigen::MatrixBase<DerivedGV> & GV,
  const Eigen::MatrixBase<Derivedside> & side,
  const TetrahedralizedGripType type,
  Eigen::PlainObjectBase<DerivedGT> & GT)
{
  const int nx = side(0);
  const int ny = side(1);
  const int nz = side(2);
  typedef typename DerivedGT::Scalar Index;
  const int m = (nx-1)*(ny-1)*(nz-1);
  // Rotationally symmetric
  int nt = -1;
  switch(type)
  {
    default: assert(false); break;
    case TETRAHEDRALIZED_GRID_TYPE_5: nt = 5; break;
    case TETRAHEDRALIZED_GRID_TYPE_6_ROTATIONAL: nt = 6; break;
  }
  GT.resize(m*nt,4);
  {
    int u = 0;
    for(int i = 0;i<nx-1;i++)
    {
      for(int j = 0;j<ny-1;j++)
      {
        for(int k = 0;k<nz-1;k++)
        {
          int v1 = (i+0)+nx*((j+0)+ny*(k+0));
          int v2 = (i+0)+nx*((j+1)+ny*(k+0));
          int v3 = (i+1)+nx*((j+0)+ny*(k+0));
          int v4 = (i+1)+nx*((j+1)+ny*(k+0));
          int v5 = (i+0)+nx*((j+0)+ny*(k+1));
          int v6 = (i+0)+nx*((j+1)+ny*(k+1));
          int v7 = (i+1)+nx*((j+0)+ny*(k+1));
          int v8 = (i+1)+nx*((j+1)+ny*(k+1));
          switch(type)
          {
            default: assert(false); break;
            case TETRAHEDRALIZED_GRID_TYPE_6_ROTATIONAL:
              // Rotationally symmetric
              GT.row(u*nt+0) << v1,v3,v8,v7;
              GT.row(u*nt+1) << v1,v8,v5,v7;
              GT.row(u*nt+2) << v1,v3,v4,v8;
              GT.row(u*nt+3) << v1,v4,v2,v8;
              GT.row(u*nt+4) << v1,v6,v5,v8;
              GT.row(u*nt+5) << v1,v2,v6,v8;
              break;
            case TETRAHEDRALIZED_GRID_TYPE_5:
            {
              // Five
              bool flip = true;
              if(((bool(i%2))^ (bool(j%2)))^ (bool(k%2)))
              {
                std::swap(v1,v3);
                std::swap(v2,v4);
                std::swap(v5,v7);
                std::swap(v6,v8);
                flip = false;
              }
              GT.row(u*nt+0) << v5,v3,v2,v1;
              GT.row(u*nt+1) << v3,v2,v8,v5;
              GT.row(u*nt+2) << v3,v4,v8,v2;
              GT.row(u*nt+3) << v3,v8,v7,v5;
              GT.row(u*nt+4) << v2,v6,v8,v5;
              if(flip)
              {
                std::swap(GT(u*nt+0,0),GT(u*nt+0,1));
                std::swap(GT(u*nt+1,0),GT(u*nt+1,1));
                std::swap(GT(u*nt+2,0),GT(u*nt+2,1));
                std::swap(GT(u*nt+3,0),GT(u*nt+3,1));
                std::swap(GT(u*nt+4,0),GT(u*nt+4,1));
              }
              break;
            }
          }
          u++;
        }
      }
    }
    assert(u == m);
  }
}

#ifdef IGL_STATIC_LIBRARY
// Explicit template instantiation
// generated by autoexplicit.sh
template void igl::tetrahedralized_grid<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, 1, 3, 1, 1, 3>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, 1, 3, 1, 1, 3> > const&, igl::TetrahedralizedGripType, Eigen::PlainObjectBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> >&);
// generated by autoexplicit.sh
#endif