import sys
from collections import deque


import networkx as nx
import matplotlib.pyplot as plt

import heapq


class Grafo:

    ######## Construtor #############
    def __init__(self, vertices, direcionado):
        # Inicializa o grafo com um número de vértices e define se é direcionado.
        self.vertices = vertices
        self.direcionado = direcionado
        self.adjacencias = {i: [] for i in range(vertices)}

    ######## Questões ############

    ### 0 - Conexo
    ## Verifica a conectividade atráves de DFS (Caso todos os vértices)
    ## Se todos os vértices forem visitados, então o grafo é conexo
    def conexo(self):
        visitados = set()

        if self.direcionado:
            # Verifica conectividade fraca: considera o grafo como não direcionado
            grafo_nao_direcionado = Grafo(self.vertices, False)
            for u in self.adjacencias:
                for v, peso, _ in self.adjacencias[u]:
                    grafo_nao_direcionado._adicionar_aresta(None, u, v, peso)
            grafo_nao_direcionado._dfs(0, visitados)
        else:
            # Verifica conectividade em grafo não direcionado
            self._dfs(0, visitados)

        return int(len(visitados) == self.vertices)

    ### 1 - Bipartido
    def bipartido(self):
        cores = [-1] * self.vertices  # -1 indica que o vértice ainda não foi colorido
        for i in range(self.vertices):
            if cores[i] == -1:  # Se o vértice ainda não foi visitado
                if not self._bfs_colore(i, cores):
                    return 0
        return 1  

    ### 2 - Euleriano
    def euleriano(self):
        if self.direcionado:
            return self._euleriano_direcionado()
        else:
            return self._euleriano_nao_direcionado()

    ### 3 - Ciclo
    def ciclo(self):
        visitados = set()
        em_processamento = set()

        for v in range(self.vertices):
            if v not in visitados:
                if self._dfs_ciclo(v, visitados, em_processamento, -1):
                    return 1
        return 0
    
    ### 4 - Calcular a quantidade de componentes conexas em um grafo não-orientado. 
    def qntd_compo_conex(self):
        if self.direcionado:
            return -1
        
        visitados = set()
        quantidade = 0

        for v in range(self.vertices):
            if v not in visitados:
                # Usa a função _dfs para marcar todos os vértices na mesma componente
                self._dfs(v, visitados) 
                quantidade += 1

        return quantidade
    
    ### 5 - Calcular a quantidade de componentes fortemente conexas em um grafo orientado. 
    def qntd_compo_fort_conex(self):
        if not self.direcionado:
            return -1

        ## Utiliza o conceito do algoritmo de Kosaraju

        empilhamento = []
        visitados = set()

        # Primeira passagem: _dfs para ordenar os vértices
        for v in range(self.vertices):
            if v not in visitados:
                self._dfs(v, visitados, empilhamento)

        # Cria o grafo transposto
        grafo_transposto = self._transpor()

        # Segunda passagem: _dfs no grafo transposto
        visitados.clear()
        quantidade_scc = 0

        while empilhamento:
            v = empilhamento.pop()
            if v not in visitados:
                grafo_transposto._dfs(v, visitados)
                quantidade_scc += 1

        return quantidade_scc
    
    ### 6 - Imprimir os vértices de articulação 
    # de um grafo não-orientado (priorizar a ordem lexicográfica dos vértices).
    def imprimir_articulacoes(self): 
        if self.direcionado:
            return print(-1)
        
        visitados = set()
        desc = [-1] * self.vertices
        low = [-1] * self.vertices
        pai = [None] * self.vertices
        articulacoes = set()
        time = [0]

        for v in range(self.vertices):
            if v not in visitados:
                self._dfs_articulacao(v, visitados, desc, low, pai, articulacoes, time)

        articulacoes_sorted = sorted(articulacoes)
        if len(articulacoes_sorted) == 0:
            return print('0 ')
        for vertice in articulacoes_sorted:
            print(vertice, end=' ')
        print('')
    
    ### 7 - Calcular quantas arestas ponte possui um grafo não-orientado. 
    def calcula_pontes(self):
        if self.direcionado:
            return -1

        visitados = set()
        disc = [-1] * self.vertices  # Tempo de descoberta dos vértices
        low = [-1] * self.vertices   # Menor tempo de descoberta alcançável
        pai = [None] * self.vertices
        pontes = set()
        tempo = [0]  # Variável para manter o tempo de descoberta

        for v in range(self.vertices):
            if v not in visitados:
                self._dfs_pontes(v, visitados, disc, low, pai, pontes, tempo)

        return len(pontes)
    
    ### 8 - Imprimir a árvore em profundidade (priorizando a ordem lexicográfica dos vértices; 0 é a origem).
    def imprimir_arvore_profundidade(self):
        visitados = set()
        arestas_usadas = []

        def dfs_modificado(v):
            visitados.add(v)
            # Ordena os vizinhos pelo identificador do vértice
            for vizinho, _, id_aresta in sorted(self.adjacencias[v], key=lambda x: x[0]):
                if vizinho not in visitados:
                    arestas_usadas.append(id_aresta)
                    dfs_modificado(vizinho)

        # Inicia a DFS a partir do vértice 0
        dfs_modificado(0)

        # Imprime os identificadores das arestas na ordem em que foram usadas
        for id_aresta in sorted(arestas_usadas):
            print(f'{id_aresta} ', end='')
        print('')

    ### 9 - Imprimir a árvore em largura (priorizando a ordem lexicográfica dos vértices; 0 é a origem).
    def imprimir_arvore_largura(self):
        if self.direcionado:
            return -1

        visitados = set()
        fila = deque([0])
        arvore = []
        visitados.add(0)

        while fila:
            v = fila.popleft()
            for vizinho, _, id_aresta in sorted(self.adjacencias[v], key=lambda x: x[0]):
                if vizinho not in visitados:
                    arvore.append(id_aresta)
                    visitados.add(vizinho)
                    fila.append(vizinho)

        return ' '.join(map(str, arvore))
    
    ### 10 - Calcular o valor final de uma Árvore Geradora Mínima
    def calcular_valor_agm(self):
        if self.direcionado:
            return -1

        # Inicializa a AGM com 0
        agm = 0

        # Utiliza o algoritmo de Kruskal para encontrar a AGM
        arestas = sorted([(peso, u, v) for u in self.adjacencias for v, peso, _ in self.adjacencias[u]])
        visitados = set()

        for peso, u, v in arestas:
            if v not in visitados or u not in visitados:
                visitados.add(u)
                visitados.add(v)
                agm += peso

        # Verifica se o grafo é conexo
        if len(visitados) != self.vertices:
            return -1

        return agm
    
    ### 11 - Imprimir a ordenação topológica (priorizando a ordem lexicográfica dos vértices)
    def ordenacao_topologica(self):
        if not self.direcionado:
            return print('-1')

        visitados = set()
        empilhamento = []

        def dfs_ordenacao(v):
            visitados.add(v)
            # Ordena os vizinhos pelo identificador do vértice para garantir a ordem lexicográfica
            for vizinho, _, _ in sorted(self.adjacencias[v], key=lambda x: x[0]):
                if vizinho not in visitados:
                    dfs_ordenacao(vizinho)
            empilhamento.append(v)

        # Executa DFS a partir de todos os vértices não visitados
        for v in range(self.vertices):
            if v not in visitados:
                dfs_ordenacao(v)

        # A ordem topológica é a inversão da ordem de empilhamento
        while empilhamento:
            print(f'{empilhamento.pop()} ', end='')
        print('')

    ### 12 - Valor do caminho mínimo entre dois vértices (para grafos não-orientados com pelo menos um peso diferente nas arestas).  
    def caminho_minimo(self):
        if self.direcionado:
            return -1

        origem = 0
        destino = self.vertices - 1
        distancias = {i: float('inf') for i in range(self.vertices)}
        distancias[origem] = 0
        prioridade = [(0, origem)]  # (distância acumulada, vértice)

        while prioridade:
            distancia_atual, atual = heapq.heappop(prioridade)

            # Se já chegamos ao destino, retornamos a distância
            if atual == destino:
                return distancia_atual

            # Se a distância atual é maior do que a registrada, ignore
            if distancia_atual > distancias[atual]:
                continue

            for vizinho, peso, _ in self.adjacencias[atual]:
                distancia = distancia_atual + peso

                if distancia < distancias[vizinho]:
                    distancias[vizinho] = distancia
                    heapq.heappush(prioridade, (distancia, vizinho))

        # Se o destino não for alcançável
        return -1
    
    ### 13 - Calcular o valor do fluxo máximo em um grafo direcionado.
    ## Utiliza  o algoritmo de Edmonds-Karp
    def fluxo_maximo(self):
        if not self.direcionado:
            return -1

        n = self.vertices
        origem = 0
        destino = n - 1

        # Inicializa a rede de fluxo
        fluxo = [[0] * n for _ in range(n)]
        capacidade = [[0] * n for _ in range(n)]

        # Adiciona as arestas do grafo à rede de fluxo
        for u in range(n):
            for v, peso, _ in self.adjacencias[u]:
                capacidade[u][v] = peso

        fluxo_maximo = 0
        while True:
            # Encontra um caminho aumentante usando BFS
            pai = [-1] * n
            fila = [origem]
            pai[origem] = origem
            while fila and pai[destino] == -1:
                u = fila.pop(0)
                for v in range(n):
                    if pai[v] == -1 and capacidade[u][v] - fluxo[u][v] > 0:
                        pai[v] = u
                        fila.append(v)

            # Se não há mais caminhos aumentantes, termina
            if pai[destino] == -1:
                break

            # Calcula o fluxo do caminho encontrado
            fluxo_caminho = float('inf')
            v = destino
            while v != origem:
                u = pai[v]
                fluxo_caminho = min(fluxo_caminho, capacidade[u][v] - fluxo[u][v])
                v = u

            # Atualiza o fluxo total e a rede de fluxo
            v = destino
            while v != origem:
                u = pai[v]
                fluxo[u][v] += fluxo_caminho
                fluxo[v][u] -= fluxo_caminho
                v = u

            fluxo_maximo += fluxo_caminho

        return fluxo_maximo

    ### 14 - Fecho transitivo para grafos direcionados.
    def fecho_transitivo(self):
        if not self.direcionado:
            return print(-1)

        visitados = set()
        # Inicia a DFS a partir do vértice 0
        self._dfs(0, visitados)

        # Retorna os vértices alcançáveis a partir do vértice 0, em ordem lexicográfica
        visitados_sorted = sorted(visitados)

        for v in visitados_sorted:
            print(f'{v} ', end='')
        print('')

    ############## Funções Auxiliares ###########

    def _adicionar_aresta(self, id_aresta, u, v, peso=1):
        # Adiciona uma aresta entre os vértices u e v com um peso opcional 
        # e o identificador da aresta.
        self.adjacencias[u].append((v, peso, id_aresta))
        if not self.direcionado:
            self.adjacencias[v].append((u, peso, id_aresta))

    def _dfs(self, v, visitados, empilhamento=None):
        visitados.add(v)
        for vizinho, _ , _ in self.adjacencias[v]:
            if vizinho not in visitados:
                self._dfs(vizinho, visitados, empilhamento)
        if empilhamento is not None:
            empilhamento.append(v)

    def _bfs_colore(self, vertice_inicial, cores):
        fila = deque([vertice_inicial])
        cores[vertice_inicial] = 0  # Começamos colorindo o primeiro vértice com 0
        while fila:
            v = fila.popleft()
            for vizinho, _ , _ in self.adjacencias[v]:
                if cores[vizinho] == -1:  # Se o vizinho ainda não foi colorido
                    cores[vizinho] = 1 - cores[v]  # Colore com a cor oposta
                    fila.append(vizinho)
                elif cores[vizinho] == cores[v]:  # Se o vizinho já tem a mesma cor, não é bipartido
                    return 0
        return 1

    def _euleriano_nao_direcionado(self):
        # Verifica se todos os vértices têm grau par e se o grafo é conexo
        if not self.conexo():
            return 0

        graus_impares = 0
        for v in self.adjacencias:
            if len(self.adjacencias[v]) % 2 != 0:
                graus_impares += 1

        if graus_impares == 0:
            return 1
        return 0

    def _euleriano_direcionado(self):
        # Verifica se todos os vértices têm o mesmo grau de entrada e saída e se o grafo é fracamente conexo
        graus_entrada = {i: 0 for i in range(self.vertices)}
        graus_saida = {i: 0 for i in range(self.vertices)}

        for u in self.adjacencias:
            for v, peso, _ in self.adjacencias[u]:
                graus_saida[u] += 1
                graus_entrada[v] += 1

        diferenca_entradas_saidas = 0
        diferenca_saidas_entradas = 0
        for v in range(self.vertices):
            if graus_saida[v] - graus_entrada[v] == 1:
                diferenca_saidas_entradas += 1
            elif graus_entrada[v] - graus_saida[v] == 1:
                diferenca_entradas_saidas += 1
            elif graus_entrada[v] != graus_saida[v]:
                return 0

        # Verificar conectividade fraca utilizando a função conexo
        if not self.conexo():
            return 0

        if diferenca_saidas_entradas == 0 and diferenca_entradas_saidas == 0:
            return 1
        return 0
    
    ## Função auxiliar da ciclo(), implementando a Busca em Profundidade (DFS) 
    # para detecção de ciclos.
    ## Marca os vértices visitados e em processamento, verificando se há revisita 
    # a algum vértice.
    def _dfs_ciclo(self, v, visitados, em_processamento, pai):
        visitados.add(v)
        em_processamento.add(v)

        for vizinho, _ , _ in self.adjacencias[v]:
            if vizinho not in visitados:
                if self._dfs_ciclo(vizinho, visitados, em_processamento, v):
                    return 1
            elif self.direcionado or vizinho != pai:
                if vizinho in em_processamento:
                    return 1

        em_processamento.remove(v)
        return 0
    
    ## Transpõe determinado grafo
    def _transpor(self):
        grafo_transposto = Grafo(self.vertices, self.direcionado)
        for u in self.adjacencias:
            for v, peso, id_aresta in self.adjacencias[u]:
                grafo_transposto.adjacencias[v].append((u, peso, id_aresta))
        return grafo_transposto

    def _dfs_articulacao(self, v, visitados, desc, low, pai, articulacoes, time):
        visitados.add(v)
        desc[v] = low[v] = time[0]
        time[0] += 1
        filhos = 0
        
        for vizinho, _ , _ in self.adjacencias[v]:  # Considera o peso da aresta, mas não o utiliza aqui
            if vizinho not in visitados:
                pai[vizinho] = v
                filhos += 1
                self._dfs_articulacao(vizinho, visitados, desc, low, pai, articulacoes, time)
                
                # Verifica e atualiza o valor de low
                low[v] = min(low[v], low[vizinho])
                
                # Condição para o vértice de articulação
                if pai[v] is None and filhos > 1:
                    articulacoes.add(v)
                if pai[v] is not None and low[vizinho] >= desc[v]:
                    articulacoes.add(v)
            elif vizinho != pai[v]:  # Atualização do valor de low
                low[v] = min(low[v], desc[vizinho])

    def _dfs_pontes(self, u, visitados, disc, low, pai, pontes, tempo):
        visitados.add(u)
        disc[u] = low[u] = tempo[0]
        tempo[0] += 1

        for v, _, _ in self.adjacencias[u]:
            if v not in visitados:
                pai[v] = u
                self._dfs_pontes(v, visitados, disc, low, pai, pontes, tempo)

                # Atualiza o valor de low de u para o menor valor alcançável
                low[u] = min(low[u], low[v])

                # Se o menor valor alcançável do vizinho for maior que o tempo de descoberta de u, é uma ponte
                if low[v] > disc[u]:
                    pontes.add((u, v))

            elif v != pai[u]:  # Atualiza low[u] para o tempo de descoberta do vizinho
                low[u] = min(low[u], disc[v])
    
    def _dfs_ordenacao_topologica(self, v, visitados, empilhamento):
        visitados.add(v)

        for vizinho, _, _ in sorted(self.adjacencias[v], key=lambda x: x[0]):
            if vizinho not in visitados:
                self._dfs_ordenacao_topologica(vizinho, visitados, empilhamento)

        empilhamento.append(v)
    
    ########################### Visualizador #######################################
    ## Utilizamos a biblioteca python networkx para realizar a lógica de plotagem
    ## e matplotlib pra plotar o grafo resultante
    def visualizar_grafo(self):
        G = nx.Graph() if not self.direcionado else nx.DiGraph()
        
        # Adicionar vértices
        G.add_nodes_from(range(self.vertices))
        
        # Adicionar arestas
        for u in self.adjacencias:
            for v, peso, id_aresta in self.adjacencias[u]:
                G.add_edge(u, v, weight=peso, id=id_aresta)
        
        # Configurar o layout
        pos = nx.spring_layout(G)
        
        # Desenhar o grafo
        plt.figure(figsize=(12, 8))
        nx.draw(G, pos, with_labels=True, node_color='lightblue', 
                node_size=500, font_size=10, font_weight='bold')
        
        # Desenhar os pesos das arestas
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        
        # Configurar e mostrar o plot
        plt.title("Grafo Visualizado")
        plt.axis('off')
        plt.tight_layout()
        plt.show()

# Função principal para interação com o usuário
def main():

    ## Pega as questões selecionadas
    questoes_selecionados = list(map(int, input().split()))

    ## Pega a quantidade de vértices e a quantidade de arestas
    vertices_arestas = list(map(int, input().split()))

    ## Pega se o grafo é direcionado ou não
    if input() == 'direcionado':
        direcionado = True
    else: 
        direcionado = False

    ## Faz um looping recebendo o id_da_aresta vertice_v vertice_u peso
    grafo = Grafo(vertices_arestas[0], direcionado)
    for _ in range(vertices_arestas[1]):
        aresta = list(map(int, input().split()))
        grafo._adicionar_aresta(aresta[0], aresta[1], aresta[2], aresta[3])

    ## Match com todas as questões digitadas
    for i in questoes_selecionados:
        match(i):
            case 0:
                print(grafo.conexo())
            case 1:
                print(grafo.bipartido())
            case 2:
                print(grafo.euleriano())
            case 3:
                print(grafo.ciclo())
            case 4:
                print(grafo.qntd_compo_conex())
            case 5:
                print(grafo.qntd_compo_fort_conex())
            case 6:
                grafo.imprimir_articulacoes()
            case 7:
                print(grafo.calcula_pontes())
            case 8:
                grafo.imprimir_arvore_profundidade()
            case 9:
                print(grafo.imprimir_arvore_largura())
            case 10:
                print(grafo.calcular_valor_agm())
            case 11:
                grafo.ordenacao_topologica()
            case 12:
                print(grafo.caminho_minimo())
            case 13:
                print(grafo.fluxo_maximo())
            case 14:
                grafo.fecho_transitivo()
            case 15:
                grafo.visualizar_grafo()
 
if __name__ == '__main__':
    main()