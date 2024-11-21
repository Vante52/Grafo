#ifndef GRAFO_H
#define GRAFO_H

#include <vector>
#include <map>
#include <queue>
#include <stack>
#include <algorithm>
#include <limits>
#include <utility> // Para std::pair
#include <set>
#include <functional> // Para std::function en Kruskal y Puentes

template<class T>
class Grafo {
private:
    std::vector<T> vertices; 
    std::vector<std::map<int, float>> aristas; 

public:
    // Constructor
    Grafo();

    // Setters
    void setVertices(const std::vector<T>& vertices);
    void setAristas(const std::vector<std::map<int, float>>& aristas);

    // Getters
    const std::vector<T>& getVertices() const;
    const std::vector<std::map<int, float>>& getAristas() const;

    // Métodos de consulta
    int cantVertices() const;
    int cantAristas() const;
    int buscarVertice(const T& ver) const;
    float buscarArista(const T &origen, const T &destino) const;
    double obtenerCosto(int id1, int id2) const;

    // Métodos de modificación
    bool insertarVertice(const T& ver);
    bool insertarArista(const T& ori, const T& des, float cos);
    bool insAristaNoDir(const T& ori, const T& des, float cos);
    bool eliminarArista(const T &origen, const T &destino);
    bool elimAristaNoDir(const T& ori, const T& des);
    bool eliminarVertice(const T& ver);

    // Métodos de análisis
    std::vector<T> vecinosVertice(const T& ver) const;
    std::vector<unsigned long> indicesVecinos(int u) const; 
    std::vector<T> DFS(const T& ver_inicial);
    std::vector<T> BFS(const T& ver_inicial);   
    std::vector<std::vector<unsigned long>> dijkstra(unsigned long i_fuente); 
    std::vector<std::vector<unsigned long>> prim(int indiceInicio) const; 
    std::vector<std::pair<T, T>> encontrarPuentes();
    bool esEuler() const; 
    std::vector<T> circuitoEuler() const;
    std::pair<std::vector<std::vector<float>>, std::vector<std::vector<int>>> floydWarshall() const;
    std::vector<std::tuple<T, T, float>> kruskal() const; 
};

#include "Grafo.cxx"
#endif // GRAFO_H
