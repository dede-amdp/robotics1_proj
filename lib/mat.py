class mat:
    def __init__(self, data:list[list[float]]):
        self.data = data
        self.n = len(data)
        self.m = len(data[0])
        self.d = None
    
    @staticmethod
    def create(n: int, m: int, value: float = 0):
        data = []
        for _ in range(n):
            row = []
            for _ in range(m):
                row.append(value)
            data.append(row)
        return mat(data)


    def t(self):
        M = mat.create(self.m, self.n)
        for i in range(self.n):
            for j in range(self.m):
                M.data[j][i] = self.data[i][j]
        return M
    
    def copy(self):
        M = mat.create(self.n, self.m)
        for i in range(self.n):
            for j in range(self.m):
                M.data[i][j] = self.data[i][j]
        return M
    
    def cut(self, r, c):
        M = mat.create(self.n-1, self.m-1)
        for i in range(self.n-1):
            for j in range(self.m-1):
                M.data[i][j] = self.data[i if i<r else i+1][j if j<c else j+1]
        return M
    
    def det(self):
        if self.n != self.m: return None
        if self.d : return self.d
        if self.n == 1: return self.data[0][0]
        if self.n == 2: return self.data[1][1]*self.data[0][0]-self.data[0][1]*self.data[1][0]
        d = 0
        for i in range(self.n):
            d += ((-1)**i)*self.data[i][0]*self.copy().cut(i,0).det()
        return d
    
    def mult(self, value):
        M = self.copy()
        for i in range(self.n):
            for j in range(self.m):
                M.data[i][j] *= value
        return M
    
    def dot(self, B):
        if self.m != B.n: return None
        M = mat.create(self.n, B.m)
        # Mij = Aik+Bkj
        for i in range(self.n):
            for j in range(B.m):
                for k in range(self.m):
                    M.data[i][j] += self.data[i][k]*B.data[k][j]
        return M


    def adj(self):
        M = mat.create(self.n, self.m)
        for i in range(self.n):
            for j in range(self.m):
                M.data[i][j] = ((-1)**(i+j))*self.copy().cut(i,j).det()
        return M

    def inv(self):
        d = self.det()
        if d == 0: return None
        if self.n ==1 : return mat([[1/self.data[0][0]]])
        return self.adj().t().mult(1/d)
    
    def __getitem__(self, key):
        if isinstance(key[0], slice):
            key = (key[1], key[0])
            M = self.t()
            return M[key]
        else:
            x,y = key
            return self.data[x][y]

    def __setitem__(self, key:tuple[int], value):
        x,y = key
        self.data[x][y] = value
    
    def __str__(self):
        return str(self.data)

    def __repr__(self):
        return str(self.data)