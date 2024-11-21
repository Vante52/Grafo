#ifndef GRAFO_CXX
#define GRAFO_CXX

#include <iostream>
#include <limits>
#include <climits>
#include <stack>
#include <queue>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>
#include <set>
#include <functional>
#include "Grafo.h"

// Implementación de la clase Grafo

// Constructor
template <class T>
Grafo<T>::Grafo() {
    // Inicialmente, no hay vértices ni aristas
}

// Setters
template <class T>
void Grafo<T>::setVertices(const std::vector<T>& vertices) {
    this->vertices = vertices;
    // Re-inicializar las aristas para coincidir con el nuevo número de vértices
    aristas.clear();
    aristas.resize(vertices.size());
}

template <class T>
void Grafo<T>::setAristas(const std::vector<std::map<int, float>>& aristas) {
    this->aristas = aristas;
}

// Getters
template <class T>
const std::vector<T>& Grafo<T>::getVertices() const {
    return this->vertices;
}

template <class T>
const std::vector<std::map<int, float>>& Grafo<T>::getAristas() const {
    return this->aristas;
}

// Cantidad de vértices
template <class T>
int Grafo<T>::cantVertices() const {
    return vertices.size();
}

// Cantidad de aristas
template <class T>
int Grafo<T>::cantAristas() const {
    int suma = 0;
    for (const std::map<int, float>& mapa : aristas) {
    suma += mapa.size();
    }
    return suma;
}

// Buscar índice de un vértice
template <class T>
int Grafo<T>::buscarVertice(const T& ver) const {
    for (size_t i = 0; i < vertices.size(); ++i) {
        if (vertices[i] == ver) return static_cast<int>(i);
    }
    return -1;
}

// Insertar un vértice
template <class T>
bool Grafo<T>::insertarVertice(const T& ver) {
    if (buscarVertice(ver) != -1) return false; // Ya existe
    vertices.push_back(ver);
    aristas.emplace_back(); // Añadir un mapa vacío para el nuevo vértice
    return true;
}

// Insertar una arista dirigida
template <class T>
bool Grafo<T>::insertarArista(const T& ori, const T& des, float cos) {
    int i_ori = buscarVertice(ori);
    int i_des = buscarVertice(des);
    if (i_ori == -1 || i_des == -1) return false; // Vértices no encontrados
    // Insertar o actualizar el costo de la arista
    aristas[i_ori][i_des] = cos;
    return true;
}

// Insertar una arista no dirigida
template <class T>
bool Grafo<T>::insAristaNoDir(const T& ori, const T& des, float cos) {
    bool res1 = insertarArista(ori, des, cos);
    bool res2 = insertarArista(des, ori, cos);
    return res1 && res2;
}

// Buscar costo de una arista
template <class T>
float Grafo<T>::buscarArista(const T &origen, const T &destino) const {
    int i_ori = buscarVertice(origen);
    int i_des = buscarVertice(destino);
    if (i_ori == -1 || i_des == -1) return -1.0f; // Indica que no existe
    auto it = aristas[i_ori].find(i_des);
    if (it != aristas[i_ori].end()) {
        return it->second;
    }
    return -1.0f; // Indica que no existe
}

// Eliminar una arista dirigida
template <class T>
bool Grafo<T>::eliminarArista(const T &origen, const T &destino) {
    int i_ori = buscarVertice(origen);
    int i_des = buscarVertice(destino);
    if (i_ori == -1 || i_des == -1) return false; // Vértices no encontrados
    auto it = aristas[i_ori].find(i_des);
    if (it != aristas[i_ori].end()) {
        aristas[i_ori].erase(it);
        return true;
    }
    return false; // Arista no encontrada
}

// Eliminar una arista no dirigida
template <class T>
bool Grafo<T>::elimAristaNoDir(const T& ori, const T& des) {
    bool res1 = eliminarArista(ori, des);
    bool res2 = eliminarArista(des, ori);
    return res1 && res2;
}

// Eliminar un vértice
template <class T>
bool Grafo<T>::eliminarVertice(const T& ver) {
    int i_ver = buscarVertice(ver);
    if (i_ver == -1) return false; // Vértice no encontrado

    // Eliminar el vértice del vector de vértices
    vertices.erase(vertices.begin() + i_ver);

    // Eliminar todas las aristas salientes de este vértice
    aristas.erase(aristas.begin() + i_ver);

    // Eliminar todas las aristas entrantes a este vértice
    for (std::vector<std::map<int, float>>::iterator it = aristas.begin(); it != aristas.end(); ++it) {
        it->erase(i_ver);
    }

    // Reajustar las claves de los mapas que sean mayores que i_ver
    for (std::vector<std::map<int, float>>::iterator it = aristas.begin(); it != aristas.end(); ++it) {
        std::map<int, float> mapa_actualizado;
        for (std::map<int, float>::const_iterator it_mapa = it->begin(); it_mapa != it->end(); ++it_mapa) {
            int dest = it_mapa->first;
            float cost = it_mapa->second;

            if (dest > i_ver) {
                mapa_actualizado[dest - 1] = cost;
            } else if (dest < i_ver) {
                mapa_actualizado[dest] = cost;
            }
            // Si dest == i_ver, ya se ha eliminado
        }
        *it = std::move(mapa_actualizado);
    }

    return true;
}


// Obtener vecinos de un vértice
template <class T>
std::vector<T> Grafo<T>::vecinosVertice(const T& ver) const {
    int indice = buscarVertice(ver);
    std::vector<T> ver_vecinos;

    if (indice != -1) {
        for (const std::pair<int, float>& arista : aristas[indice]) {
            ver_vecinos.push_back(vertices[arista.first]);
        }
    }

    // Ordenar los vecinos dependiendo de su tipo de dato
    std::sort(ver_vecinos.begin(), ver_vecinos.end());
    return ver_vecinos;
}

template <class T>
std::vector<unsigned long> Grafo<T>::indicesVecinos(int u) const {
    std::vector<unsigned long> vecinos;
    if (u < 0 || u >= cantVertices()) return vecinos;
    for (const std::pair<int, float>& arista : aristas[u]) {
        vecinos.push_back(arista.first);
    }
    return vecinos;
}

// Implementación de DFS
template <class T>
std::vector<T> Grafo<T>::DFS(const T& ver_inicial) {
    std::vector<bool> ver_visitados(cantVertices(), false);
    std::vector<T> caminoDFS;
    std::stack<int> pila_ver;

    int indice_inicial = buscarVertice(ver_inicial);
    if (indice_inicial == -1) {
        // Vértice no encontrado, retornar camino vacío
        return caminoDFS;
    }

    pila_ver.push(indice_inicial);

    while (!pila_ver.empty()) {
        int ver_actual = pila_ver.top();
        pila_ver.pop();

        if (!ver_visitados[ver_actual]) {
            caminoDFS.push_back(vertices[ver_actual]);
            ver_visitados[ver_actual] = true;

            // Insertar vecinos en orden inverso para mantener el orden correcto en el stack
            std::vector<int> vecinos_indices;
            for (const std::pair<int, float>& arista : aristas[ver_actual]) {
                vecinos_indices.push_back(arista.first);
            }

            // Ordenar en orden descendente para que al insertar en el stack, se procesen en orden ascendente
            std::sort(vecinos_indices.begin(), vecinos_indices.end(), [&](int a, int b) {
                return vertices[a] > vertices[b];
            });

            for (const int vecino : vecinos_indices) {
                if (!ver_visitados[vecino]) {
                    pila_ver.push(vecino);
                }
            }
        }
    }

    return caminoDFS;
}


// Implementación de BFS
template <class T>
std::vector<T> Grafo<T>::BFS(const T& ver_inicial) {
    std::vector<bool> ver_visitados(cantVertices(), false);
    std::vector<T> caminoBFS;
    std::queue<int> cola_ver;

    int indice_inicial = buscarVertice(ver_inicial);
    if (indice_inicial == -1) {
        // Vértice no encontrado, retornar camino vacío
        return caminoBFS;
    }

    cola_ver.push(indice_inicial);
    ver_visitados[indice_inicial] = true;

    while (!cola_ver.empty()) {
        int ver_actual = cola_ver.front();
        cola_ver.pop();
        caminoBFS.push_back(vertices[ver_actual]);

        for (const std::pair<int, float>& arista : aristas[ver_actual]) {
            if (!ver_visitados[arista.first]) {
                cola_ver.push(arista.first);
                ver_visitados[arista.first] = true;
            }
        }
    }

    return caminoBFS;
}

// Implementación de Dijkstra
template <class T>
std::vector<std::vector<unsigned long>> Grafo<T>::dijkstra(unsigned long i_fuente) {
    int numVertices = cantVertices();
    std::vector<float> dist(numVertices, std::numeric_limits<float>::max());
    std::vector<unsigned long> pred(numVertices, static_cast<unsigned long>(-1));
    std::vector<unsigned long> q;

    std::vector<std::vector<unsigned long>> todasRutas;
    std::vector<unsigned long> rutita;
    std::vector<unsigned long> rutitaInversa;
    std::vector<unsigned long> vecinos;

    // Inicializar la cola con todos los vértices
    for (unsigned long i = 0; i < cantVertices(); i++) {
        q.push_back(i);
    }

    // Verificar que i_fuente es un índice válido
    if (i_fuente >= cantVertices()) {
        return {}; // Retornar vector vacío si el índice es inválido
    }

    pred[i_fuente] = i_fuente;
    dist[i_fuente] = 0.0f;

    // Conjunto de vértices pendientes
    std::set<unsigned long> vecinosPendientes;
    vecinosPendientes.insert(i_fuente);

    // Bucle principal de Dijkstra
    while (!vecinosPendientes.empty()) {
        unsigned long u = static_cast<unsigned long>(-1);
        float pesoMinimo = std::numeric_limits<float>::max();

        // Encontrar el vértice `u` con el menor peso de conexión no incluido en el árbol
        for (std::set<unsigned long>::iterator it = vecinosPendientes.begin(); it != vecinosPendientes.end(); ++it) {
            if (dist[*it] < pesoMinimo) {
                pesoMinimo = dist[*it];
                u = *it;
            }
        }

        // Si no se encuentra un vértice, terminar
        if (u == static_cast<unsigned long>(-1)) break;

        vecinosPendientes.erase(u); // Eliminar `u` del conjunto

        // Construir la ruta desde el inicio hasta el vértice `u` en `todasRutas`
        std::vector<unsigned long> ruta;
        unsigned long actual = u;
        while (pred[actual] != actual) {
            ruta.push_back(actual);
            actual = pred[actual];
        }
        ruta.push_back(i_fuente); // Agregar el inicio al final
        std::reverse(ruta.begin(), ruta.end());
        todasRutas.push_back(ruta);

        // Explorar vecinos de `u` y actualizar pesos mínimos
        vecinos = indicesVecinos(u);
        for (unsigned long v : vecinos) {
            float pesoArista = aristas[u][v];
            if (dist[u] + pesoArista < dist[v]) {
                dist[v] = dist[u] + pesoArista;
                pred[v] = u;
                vecinosPendientes.insert(v);
            }
        }
    }

    // Reconstruir las rutas para los vértices que son alcanzables
    for (unsigned long i = 0; i < cantVertices(); i++) {
        if (pred[i] != static_cast<unsigned long>(-1)) {
            if (i != i_fuente) {
                rutitaInversa.push_back(i);
            }
            rutitaInversa.push_back(pred[i]);
            unsigned long predecesor = pred[i];
            while (predecesor != i_fuente) {
                rutitaInversa.push_back(predecesor);
                predecesor = pred[predecesor];
            }
            rutita.push_back(i_fuente);
            for (std::vector<unsigned long>::reverse_iterator it = rutitaInversa.rbegin(); it != rutitaInversa.rend(); ++it) {
                rutita.push_back(*it);
            }
            todasRutas.push_back(rutita);
            rutita.clear();
            rutitaInversa.clear();
        }
    }

    return todasRutas;
}


// Implementación de obtenerCosto
template <class T>
double Grafo<T>::obtenerCosto(int id1, int id2) const {
    if (id1 < 0 || id1 >= cantVertices() || id2 < 0 || id2 >= cantVertices()) {
        return -1.0; // Valor inválido para indicar que no existe
    }

    for (std::map<int, float>::const_iterator it = aristas[id1].begin(); it != aristas[id1].end(); ++it) {
        if (it->first == id2) {
            return it->second; // Devolver el costo del segundo coso que es el que es
        }
    }

    return -1.0; // Indica que no existe
}


// Implementación de encontrarPuentes
template <class T>
std::vector<std::pair<T, T>> Grafo<T>::encontrarPuentes() {
    int numVertices = cantVertices();
    std::vector<int> tiempos(numVertices, -1);
    std::vector<int> menorTiempo(numVertices, -1);
    std::vector<int> nodoPadre(numVertices, -1);
    int tiempoGlobal = 0;
    std::vector<std::pair<T, T>> puentes;

    // Función lambda para DFS y encontrar puentes
    std::function<void(int)> dfsPuentes = [&](int u) {
        tiempos[u] = menorTiempo[u] = ++tiempoGlobal;

        for (const std::pair<int, float>& arista : aristas[u]) {
            int v = arista.first;
            if (v == nodoPadre[u]) continue;

            if (tiempos[v] == -1) { // Si v no ha sido visitado
                nodoPadre[v] = u;
                dfsPuentes(v);

                // Actualizar menorTiempo de u
                menorTiempo[u] = std::min(menorTiempo[u], menorTiempo[v]);

                // Si el menor tiempo de v es mayor que el tiempo de u, entonces es un puente
                if (menorTiempo[v] > tiempos[u]) {
                    puentes.emplace_back(std::make_pair(vertices[u], vertices[v]));
                }
            } else { // Si v ya ha sido visitado y no es el padre, actualizar menorTiempo
                menorTiempo[u] = std::min(menorTiempo[u], tiempos[v]);
            }
        }
    };

    // Llamar a DFS para cada componente conectada
    for (int i = 0; i < numVertices; ++i) {
        if (tiempos[i] == -1) {
            dfsPuentes(i);
        }
    }

    return puentes;
}


// Implementación de esEuler
template <class T>
bool Grafo<T>::esEuler() const {
    int numVertices = cantVertices();
    int imparCont = 0;

    for (int i = 0; i < numVertices; ++i) {
        int grado = aristas[i].size();
        if (grado % 2 != 0) {
            imparCont++;
        }
    }

    if (imparCont == 0 || imparCont == 2) {
        // El grafo tiene un circuito de Euler (0 impares) o un camino de Euler (2 impares
        return true;
    } else {
        // El grafo no tiene un camino ni un circuito de Euler
        return false;
    }
}

// Implementación de circuitoEuler
template <class T>
std::vector<T> Grafo<T>::circuitoEuler() const {
    std::vector<T> recorridoEuler;

    if (!esEuler()) {
        // El grafo no es Euleriano, retornar recorrido vacío
        return recorridoEuler;
    }

    // Copiar la lista de adyacencia para modificarla durante el algoritmo
    std::vector<std::map<int, float>> aristasCopia = aristas;
    std::stack<int> pila;
    int inicio = 0;

    // Encontrar el vértice de inicio
    for (int i = 0; i < cantVertices(); ++i) {
        if (aristasCopia[i].size() % 2 != 0) {
            inicio = i;
            break;
        }
    }

    pila.push(inicio);

    while (!pila.empty()) {
        int u = pila.top();

        if (!aristasCopia[u].empty()) {
            std::map<int, float>::iterator it = aristasCopia[u].begin();
            int v = it->first;
            // Eliminar la arista u - v
            aristasCopia[u].erase(it);
            // Eliminar la arista v - u si existe (para grafos no dirigidos)
            aristasCopia[v].erase(u);
            pila.push(v);
        } else {
            recorridoEuler.push_back(vertices[u]);
            pila.pop();
        }
    }

    return recorridoEuler;
}


// Implementación de Floyd-Warshall
template <class T>
std::pair<std::vector<std::vector<float>>, std::vector<std::vector<int>>> Grafo<T>::floydWarshall() const {
    int n = cantVertices();
    std::vector<std::vector<float>> D(n, std::vector<float>(n, std::numeric_limits<float>::infinity()));
    std::vector<std::vector<int>> P(n, std::vector<int>(n, -1));

    // Inicializar la matriz de distancias y predecesores
    for (int i = 0; i < n; ++i) {
        D[i][i] = 0.0f;
        for (std::map<int, float>::const_iterator it = aristas[i].begin(); it != aristas[i].end(); ++it) {
            int j = it->first;
            float cost = it->second;
            D[i][j] = cost;
            P[i][j] = i;
        }
    }

    // Algoritmo de Floyd-Warshall
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (D[i][k] + D[k][j] < D[i][j]) {
                    D[i][j] = D[i][k] + D[k][j];
                    P[i][j] = P[k][j];
                }
            }
        }
    }

    return std::make_pair(D, P);
}


// Implementación de Kruskal
template <class T>
std::vector<std::tuple<T, T, float>> Grafo<T>::kruskal() const {
    int numVertices = cantVertices();
    std::vector<std::tuple<float, int, int>> aristasGrafo; // (peso, origen, destino)
    std::vector<std::tuple<T, T, float>> arbol; // Árbol de recubrimiento mínimo (origen, destino, peso)

    // Recolectar todas las aristas del grafo
    for (int u = 0; u < numVertices; ++u) {
        for (std::map<int, float>::const_iterator it = aristas[u].begin(); it != aristas[u].end(); ++it) {
            int v = it->first;
            float peso = it->second;
            if (u < v) { // Evitar duplicados en grafos no dirigidos
                aristasGrafo.emplace_back(std::make_tuple(peso, u, v));
            }
        }
    }

    // Ordenar las aristas por peso ascendente
    std::sort(aristasGrafo.begin(), aristasGrafo.end());

    // Estructura para manejar conjuntos disjuntos (Union-Find)
    std::vector<int> padre(numVertices);
    for (int i = 0; i < numVertices; ++i) {
        padre[i] = i;
    }

    // Función para encontrar el representante de un conjunto
    std::function<int(int)> encontrarConjunto = [&](int v) -> int {
        if (padre[v] != v)
            padre[v] = encontrarConjunto(padre[v]);
        return padre[v];
    };

    // Función para unir dos conjuntos
    std::function<void(int, int)> unirConjuntos = [&](int u, int v) {
        int set_u = encontrarConjunto(u);
        int set_v = encontrarConjunto(v);
        if (set_u != set_v) {
            padre[set_u] = set_v;
        }
    };

    // Iterar sobre las aristas y construir el árbol de recubrimiento mínimo
    for (std::vector<std::tuple<float, int, int>>::const_iterator it = aristasGrafo.begin(); it != aristasGrafo.end(); ++it) {
        float peso = std::get<0>(*it);
        int u = std::get<1>(*it);
        int v = std::get<2>(*it);

        if (encontrarConjunto(u) != encontrarConjunto(v)) {
            arbol.emplace_back(std::make_tuple(vertices[u], vertices[v], peso));
            unirConjuntos(u, v);
        }
    }

    return arbol;
}



template <class T>
std::vector<std::vector<unsigned long>> Grafo<T>::prim(int indiceInicio) const {
    int numVertices = cantVertices();
    std::vector<bool> enArbol(numVertices, false);  // Marca si el vértice está en el árbol
    std::vector<float> minPeso(numVertices, std::numeric_limits<float>::max());  // Peso mínimo
    std::vector<int> padre(numVertices, -1);  // Almacena el vértice padre de cada vértice

    // Verificar que el índice de inicio es válido
    if (indiceInicio < 0 || indiceInicio >= numVertices) {
        return {};
    }

    minPeso[indiceInicio] = 0.0f;
    std::vector<std::vector<unsigned long>> todasRutas(numVertices);  // Almacena todas las rutas

    // Conjunto de vértices pendientes
    std::set<int> vecinosPendientes;
    vecinosPendientes.insert(indiceInicio);

    // Bucle principal de Prim para construir el árbol de expansión mínima
    while (!vecinosPendientes.empty()) {
        int u = -1;
        float pesoMinimo = std::numeric_limits<float>::max();

        // Encontrar el vértice `u` con el menor peso de conexión no incluido en el árbol
        for (std::set<int>::iterator it = vecinosPendientes.begin(); it != vecinosPendientes.end(); ++it) {
            if (minPeso[*it] < pesoMinimo) {
                pesoMinimo = minPeso[*it];
                u = *it;
            }
        }

        // Si no se encuentra un vértice, terminar
        if (u == -1) break;

        vecinosPendientes.erase(u);  // Eliminar `u` del conjunto
        enArbol[u] = true;           // Marcar `u` como parte del árbol

        // Construir la ruta desde el inicio hasta el vértice `u` en `todasRutas`
        std::vector<unsigned long> ruta;
        unsigned long actual = u;
        while (actual != static_cast<unsigned long>(-1)) {
            ruta.push_back(actual);
            actual = padre[actual];
        }
        std::reverse(ruta.begin(), ruta.end());
        todasRutas[u] = ruta;

        // Explorar vecinos de `u` y actualizar pesos mínimos
        for (std::map<int, float>::const_iterator it = aristas[u].begin(); it != aristas[u].end(); ++it) {
            int v = it->first;
            float pesoArista = it->second;
            if (!enArbol[v] && pesoArista < minPeso[v]) {
                minPeso[v] = pesoArista;
                padre[v] = u;
                vecinosPendientes.insert(v);
            }
        }
    }

    return todasRutas;
}


#endif // GRAFO_CXX