  template<typename Pixel>
  static void doBin_NxN(const Pixel* source, Pixel* target, int x, int y, int N, bool avg)
  {
    uint32_t* next = reinterpret_cast<uint32_t*>(target);

    // accumulate 8bit sums as 16bit
    memset(next, 0, x*y);
    for (int i = 0; i < x; ++i) {
        for (int j = 0; j < y; ++j) {
            next[(j/N)*(x/N) + (i/N)] += source[j*x + i];
        }
    }
    // clip (bin) or divide (avg) 16bit sums back to 8bit
    for (int i=0; i<(x/N)*(y/N); ++i)
    {
        target[i] = std::min((uint32_t)std::numeric_limits<Pixel>::max(), (next[i] / (avg?(N*N):1)));
    }
  }
  template <typename Pixel>
  static void oaconvolve(const Pixel* source, Pixel* target, int x, int y, const int8_t (&k)[1], double factor, double bias)
  {
    for(int i = 0; i < x; i++) {
      for(int j = 0; j < y; j++)
      {
        int64_t val = source[ j * x +  i   ] * k[0];

        //truncate values smaller than zero and larger than 255
        target[j * x + i] = std::min(std::max(int(factor * val + bias), 0),
            (int)std::numeric_limits<Pixel>::max());
      }
    }
  }

    // helper routines to wrap pixel position if <0 or >=dim
    // caller must call with x or y correctly
    static int up(int p, int d, int n = 1)    { return (p-n + d) % d; }
    static int down(int p, int d, int n = 1)  { return (p+n + d) % d; }
    static int left(int p, int d, int n = 1)  { return up(p,d,n);     }
    static int right(int p, int d, int n = 1) { return down(p,d,n);   }

  template <typename Pixel>
  static void oaconvolve(const Pixel* source, Pixel* target, int x, int y, const int8_t (&k)[9], double factor, double bias)
  {
    for(int j = 0; j < y; j++)
    for(int i = 0; i < x; i++) {
      int l = left(i,x);
      int r = right(i,x);
      int u = up(j,y);
      int d = down(j,y);

      int64_t val =
        source[u * x + l] * k[0] +
        source[u * x + i] * k[1] +
        source[u * x + r] * k[2] +

        source[j * x + l] * k[3] +
        source[j * x + i] * k[4] +
        source[j * x + r] * k[5] +

        source[d * x + l] * k[6] +
        source[d * x + i] * k[7] +
        source[d * x + r] * k[8];

      //truncate values smaller than zero and larger than 255
      target[j * x + i] = std::min(std::max(int(factor * val + bias), 0),
        (int)std::numeric_limits<Pixel>::max());
    }
  }

  template <typename Pixel>
  static void oaconvolve(const Pixel* source, Pixel* target, int x, int y, const int8_t (&k)[25], double factor, double bias)
  {
    for(int j = 0; j < y; j++)
    for(int i = 0; i < x; i++) {
      int l = left(i,x);
      int r = right(i,x);
      int u = up(j,y);
      int d = down(j,y);
      int l2 = left(i,x,2);
      int r2 = right(i,x,2);
      int u2 = up(j,y,2);
      int d2 = down(j,y,2);

      int64_t val =
        source[u2 * x + l2] * k[0] +
        source[u2 * x + l ] * k[1] +
        source[u2 * x + i ] * k[2] +
        source[u2 * x + r ] * k[3] +
        source[u2 * x + r2] * k[4] +

        source[u  * x + l2] * k[5] +
        source[u  * x + l ] * k[6] +
        source[u  * x + i ] * k[7] +
        source[u  * x + r ] * k[8] +
        source[u  * x + r2] * k[9] +

        source[j  * x + l2] * k[10] +
        source[j  * x + l ] * k[11] +
        source[j  * x + i ] * k[12] +
        source[j  * x + r ] * k[13] +
        source[j  * x + r2] * k[14] +

        source[d  * x + l2] * k[15] +
        source[d  * x + l ] * k[16] +
        source[d  * x + i ] * k[17] +
        source[d  * x + r ] * k[18] +
        source[d  * x + r2] * k[19] +

        source[d2 * x + l2] * k[20] +
        source[d2 * x + l ] * k[21] +
        source[d2 * x + i ] * k[22] +
        source[d2 * x + r ] * k[23] +
        source[d2 * x + r2] * k[24];

      //truncate values smaller than zero and larger than 255
      target[j * x + i] = std::min(std::max(int(factor * val + bias), 0),
          (int)std::numeric_limits<Pixel>::max());
    }
  }

  /*
   * Adaptive Digital Pixel Binning
   * https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4541814/
   *
   * G       input image
   * F       output image
   * W       image width (eg 640)
   * H       image height (eg 480)
   * Rb      maximum binning ratio (eg 4)
   * lambda  noise suppression sensitivity (eg 16.0, 1.0)
   * mu      pixel depth?? (eg 255)
   *
   * #include <algorithm>; // std::min, std::max, std::sort
   * #include <limits>; // std::numeric_limits
   */
  struct AbsCompare {
    bool operator()(int a,int b){return abs(a)<abs(b);}          // compare by absolute value
  };
  template <typename Pixel>
  void oaadpb(const Pixel* G, Pixel* F, int W, int H, int Rb, double lambda, int mu=std::numeric_limits<Pixel>::max())
  {
    double* const HG=new double[W*H];                            // allocate 3x3 average buffer
    double max=0;                                                // find max average value
    for(int y=0;y<H;++y)for(int x=0;x<W;++x){                    // iterate all pixels
      const int L=(x-1+W)%W,R=(x+1+W)%W,U=(y-1+H)%H,D=(y+1+H)%H; // wrap at edges
      const double avg=(G[U*W+L]+G[U*W+x]+G[U*W+R]+              // convolve 3x3 average kernel
                        G[y*W+L]+G[y*W+x]+G[y*W+R]+              // ...
                        G[D*W+L]+G[D*W+x]+G[D*W+R])/9.0;         // ...
      HG[y*W+x]=avg;if(avg>max)max=avg;}                         // store average and find max
    for(int y=0;y<H;++y)for(int x=0;x<W;++x){                    // iterate all pixels
      const double hg=HG[y*W+x],t=hg/max,                        // average/fractional pixel values
                   r=1+(1-t)*(Rb-1);                             // optimal binning ratio
      const Pixel g=G[y*W+x];                                    // center pixel value
      const int L=(x-1+W)%W,R=(x+1+W)%W,U=(y-1+H)%H,D=(y+1+H)%H; // wrap at edges
      int d[9]={g-G[U*W+L],g-G[U*W+x],g-G[U*W+R],                // calculate differences
                g-G[y*W+L],g-G[y*W+x],g-G[y*W+R],                // ...
                g-G[D*W+L],g-G[D*W+x],g-G[D*W+R]};               // ...
      std::sort(d,d+9,AbsCompare());                             // sort differences, abs(a)<abs(b)
      double bc=0,bu=0;                                          // accumulators for convolution
      for(int q=1;q<=9;++q){                                     // iterate sorted differences
        const Pixel s=g-d[q-1];                                  // original pixel value
        const double contrib=r-(q-1);                            // calculate contribution
        bc+=contrib>1?s:contrib>0?(s*contrib):0;                 // convolve context kernel
        bu+=q==1?s:((r-1.0)/8.0)*s;}                             // convolve uniform kernel
      const double clip=std::numeric_limits<Pixel>::max();       // saturation value
      bc=std::max(0.0,std::min(clip,bc));
      bu=std::max(0.0,std::min(clip,bu));
      const double gamma=abs(hg-g)/lambda,                       // combination coefficient
                   b=(1.0-gamma)*bc+gamma*bu,                    // denoised pixel value
                   w=(b/(Rb-1.0)+g/2.0)/mu,                      // blending coefficient
                   f=(1.0-w)*b+w*g;                              // blend for anti-saturation
      F[y*W+x]=std::min(clip,1.2*f);}                            // final pixel value
    delete[](HG);                                                // clean up
  } 
